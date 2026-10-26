// Copyright (c) 2026 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

// Simulated camera that plays back image files: a single image, or a directory
// of frames (e.g. a recorded session) at the recorded cadence. Playback is
// controlled at runtime through a SimController handle, which is shared with
// whatever control surface the application provides. For testing without a
// camera or a night sky.

use std::cmp;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use async_trait::async_trait;
use canonical_error::{failed_precondition_error, invalid_argument_error,
                      CanonicalError};
use image::{GrayImage, ImageReader};
use log::{info, warn};
use tokio::task::JoinHandle;

use cedar_detect::image_funcs::bin_2x2;
use crate::abstract_camera::{AbstractCamera, CaptureParams, CapturedImage,
                             Gain, Offset};

const IMAGE_EXTENSIONS: [&str; 6] = ["png", "bmp", "jpg", "jpeg", "tif", "tiff"];
const MAX_SPEED: f64 = 100.0;

#[derive(Clone, Debug)]
struct SimFrame {
    path: PathBuf,
    // Exposure recorded for this frame, if known (from manifest.csv or a
    // "_<N>ms_" filename component). Sets the playback cadence.
    exposure: Option<Duration>,
}

struct SimState {
    // All sources must resolve to a path under `root`.
    root: PathBuf,
    // The current source (file or directory), relative to `root`.
    source: PathBuf,
    frames: Vec<SimFrame>,
    // Bumped whenever `frames` is replaced.
    generation: u64,
    index: usize,
    playing: bool,
    speed: f64,
    looping: bool,
    // Fixed by the initial source; later sources must match.
    width: u32,
    height: u32,
    // Most recently emitted frame id, for status reporting.
    frame_id: i32,
}

/// Snapshot of the simulator's playback state.
#[derive(Clone, Debug, PartialEq)]
pub struct SimStatus {
    pub source: String,
    pub frame_name: String,
    pub index: usize,
    pub count: usize,
    pub playing: bool,
    pub speed: f64,
    pub looping: bool,
    pub width: u32,
    pub height: u32,
    pub frame_id: i32,
}

/// Cloneable handle for controlling a SimCamera's playback. Never touches the
/// camera itself, so it can be used while the camera is locked mid-capture.
#[derive(Clone)]
pub struct SimController {
    state: Arc<Mutex<SimState>>,
}

impl SimController {
    /// `source` is an image file or a directory of image files; a relative
    /// `source` is resolved against `root`, and it must lie under `root`.
    /// Playback starts paused at the first frame.
    pub fn new(root: &Path, source: &Path) -> Result<Self, CanonicalError> {
        let root = root.canonicalize().map_err(|e| invalid_argument_error(
            &format!("Bad sim root {:?}: {}", root, e)))?;
        let (rel_source, frames) = load_source(&root, source)?;
        let (width, height) = check_dimensions(&frames, None)?;
        info!("Simulated camera: {:?}, {} frame(s) of {}x{}",
              rel_source, frames.len(), width, height);
        Ok(SimController{state: Arc::new(Mutex::new(SimState{
            root, source: rel_source, frames, generation: 0, index: 0,
            playing: false, speed: 1.0, looping: true, width, height,
            frame_id: 0}))})
    }

    /// Switches to a new file or directory, positioned at `index`. The new
    /// frames must have the same dimensions as the initial source. Playing
    /// state, speed, and looping are retained.
    pub fn set_source(&self, source: &Path, index: Option<usize>)
                      -> Result<SimStatus, CanonicalError> {
        let (root, width, height) = {
            let state = self.state.lock().unwrap();
            (state.root.clone(), state.width, state.height)
        };
        // Scan outside the lock; reading many image headers can take a while.
        let (rel_source, frames) = load_source(&root, source)?;
        check_dimensions(&frames, Some((width, height)))?;
        let index = index.unwrap_or(0);
        if index >= frames.len() {
            return Err(invalid_argument_error(&format!(
                "Index {} out of range for {} frame(s)", index, frames.len())));
        }
        let mut state = self.state.lock().unwrap();
        info!("Simulated camera: switching to {:?}, {} frame(s)",
              rel_source, frames.len());
        state.source = rel_source;
        state.frames = frames;
        state.generation += 1;
        state.index = index;
        Ok(status_of(&state))
    }

    pub fn play(&self) -> SimStatus {
        let mut state = self.state.lock().unwrap();
        if !state.looping && state.index + 1 == state.frames.len() {
            state.index = 0;  // Replay from the start.
        }
        state.playing = true;
        status_of(&state)
    }

    pub fn pause(&self) -> SimStatus {
        let mut state = self.state.lock().unwrap();
        state.playing = false;
        status_of(&state)
    }

    /// Moves `n` frames forward (or backward if negative), wrapping around.
    pub fn step(&self, n: i64) -> SimStatus {
        let mut state = self.state.lock().unwrap();
        let count = state.frames.len() as i64;
        state.index = (state.index as i64 + n).rem_euclid(count) as usize;
        status_of(&state)
    }

    pub fn seek(&self, index: usize) -> Result<SimStatus, CanonicalError> {
        let mut state = self.state.lock().unwrap();
        if index >= state.frames.len() {
            return Err(invalid_argument_error(&format!(
                "Index {} out of range for {} frame(s)",
                index, state.frames.len())));
        }
        state.index = index;
        Ok(status_of(&state))
    }

    /// Playback rate multiplier; 2.0 plays twice as fast as recorded.
    pub fn set_speed(&self, speed: f64) -> Result<SimStatus, CanonicalError> {
        if !(speed > 0.0 && speed <= MAX_SPEED) {
            return Err(invalid_argument_error(&format!(
                "Speed must be in (0, {}], got {}", MAX_SPEED, speed)));
        }
        let mut state = self.state.lock().unwrap();
        state.speed = speed;
        Ok(status_of(&state))
    }

    /// Whether playback wraps to the first frame after the last one, rather
    /// than pausing there.
    pub fn set_looping(&self, looping: bool) -> SimStatus {
        let mut state = self.state.lock().unwrap();
        state.looping = looping;
        status_of(&state)
    }

    pub fn status(&self) -> SimStatus {
        status_of(&self.state.lock().unwrap())
    }
}

fn status_of(state: &SimState) -> SimStatus {
    SimStatus{
        source: state.source.to_string_lossy().to_string(),
        frame_name: state.frames[state.index].path.file_name().unwrap()
            .to_string_lossy().to_string(),
        index: state.index,
        count: state.frames.len(),
        playing: state.playing,
        speed: state.speed,
        looping: state.looping,
        width: state.width,
        height: state.height,
        frame_id: state.frame_id,
    }
}

// Resolves `source` under `root` and lists its frames. Returns the source
// relative to `root`.
fn load_source(root: &Path, source: &Path)
               -> Result<(PathBuf, Vec<SimFrame>), CanonicalError> {
    let joined = root.join(source);  // No-op join if `source` is absolute.
    let path = joined.canonicalize().map_err(|e| invalid_argument_error(
        &format!("Bad sim source {:?}: {}", source, e)))?;
    let rel_source = match path.strip_prefix(root) {
        Ok(rel) => rel.to_path_buf(),
        Err(_) => {
            return Err(invalid_argument_error(&format!(
                "Sim source {:?} is outside of sim root {:?}", source, root)));
        }
    };
    let frames = if path.is_dir() {
        list_dir_frames(&path)?
    } else {
        vec![SimFrame{exposure: exposure_from_filename(&path), path}]
    };
    if frames.is_empty() {
        return Err(failed_precondition_error(&format!(
            "No image files in {:?}", source)));
    }
    Ok((rel_source, frames))
}

// Frames of a directory: in manifest.csv order if present (as written by the
// benchmark capture), else image files sorted by name.
fn list_dir_frames(dir: &Path) -> Result<Vec<SimFrame>, CanonicalError> {
    let manifest = dir.join("manifest.csv");
    if manifest.is_file() {
        return read_manifest(dir, &manifest);
    }
    let entries = std::fs::read_dir(dir).map_err(|e| invalid_argument_error(
        &format!("Cannot read {:?}: {}", dir, e)))?;
    let mut paths: Vec<PathBuf> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_file() && has_image_extension(p))
        .collect();
    paths.sort();
    Ok(paths.into_iter()
       .map(|path| SimFrame{exposure: exposure_from_filename(&path), path})
       .collect())
}

// Minimal CSV reading: needs a header with a `filename` column; uses
// `exposure_ms` if present. Fields are assumed not to contain commas.
fn read_manifest(dir: &Path, manifest: &Path)
                 -> Result<Vec<SimFrame>, CanonicalError> {
    let text = std::fs::read_to_string(manifest).map_err(|e| {
        invalid_argument_error(&format!("Cannot read {:?}: {}", manifest, e))
    })?;
    let mut lines = text.lines();
    let header: Vec<&str> =
        lines.next().unwrap_or("").split(',').map(|h| h.trim()).collect();
    let Some(filename_col) = header.iter().position(|h| *h == "filename") else {
        return Err(failed_precondition_error(&format!(
            "{:?} has no 'filename' column", manifest)));
    };
    let exposure_col = header.iter().position(|h| *h == "exposure_ms");
    let mut frames = Vec::new();
    for line in lines.filter(|l| !l.trim().is_empty()) {
        let fields: Vec<&str> = line.split(',').map(|f| f.trim()).collect();
        let Some(filename) = fields.get(filename_col) else { continue };
        let path = dir.join(filename);
        if !path.is_file() {
            warn!("{:?} lists missing file {:?}", manifest, filename);
            continue;
        }
        let exposure = exposure_col
            .and_then(|c| fields.get(c))
            .and_then(|f| f.parse::<f64>().ok())
            .filter(|ms| *ms > 0.0)
            .map(|ms| Duration::from_secs_f64(ms / 1000.0))
            .or_else(|| exposure_from_filename(&path));
        frames.push(SimFrame{path, exposure});
    }
    Ok(frames)
}

fn has_image_extension(path: &Path) -> bool {
    path.extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .is_some_and(|e| IMAGE_EXTENSIONS.contains(&e.as_str()))
}

// Parses an exposure from a filename component like "img_000003_70ms_...".
fn exposure_from_filename(path: &Path) -> Option<Duration> {
    let stem = path.file_stem()?.to_string_lossy().to_string();
    stem.split('_')
        .filter_map(|part| part.strip_suffix("ms"))
        .filter_map(|ms| ms.parse::<u64>().ok())
        .find(|ms| *ms > 0)
        .map(Duration::from_millis)
}

// Reads only image headers. All frames must share one size, which must equal
// `expected` if given. Returns that size.
fn check_dimensions(frames: &[SimFrame], expected: Option<(u32, u32)>)
                    -> Result<(u32, u32), CanonicalError> {
    let mut size = expected;
    for frame in frames {
        let dims = image::image_dimensions(&frame.path).map_err(|e| {
            invalid_argument_error(&format!(
                "Cannot read image {:?}: {}", frame.path, e))
        })?;
        match size {
            None => size = Some(dims),
            Some(s) if s != dims => {
                return Err(invalid_argument_error(&format!(
                    "Image {:?} is {}x{}, expected {}x{}",
                    frame.path, dims.0, dims.1, s.0, s.1)));
            }
            _ => (),
        }
    }
    Ok(size.unwrap())
}

fn decode(path: &Path) -> Result<GrayImage, String> {
    let img = ImageReader::open(path).map_err(|e| e.to_string())?
        .decode().map_err(|e| e.to_string())?;
    Ok(img.to_luma8())
}

// Identifies a frame across source switches.
type FrameKey = (u64, usize);  // (generation, index)

struct DecodedFrame {
    key: FrameKey,
    image: Arc<GrayImage>,
    binned_image: Arc<GrayImage>,
    exposure: Option<Duration>,
}

pub struct SimCamera {
    controller: SimController,
    exposure_duration: Duration,

    offset: Offset,
    gain: Gain,

    // Zero means go fast as camera frames are available.
    update_interval: Duration,

    current: DecodedFrame,
    // Decoding of the frame expected next, started after each capture.
    prefetch: Option<(FrameKey, JoinHandle<Result<GrayImage, String>>)>,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<CapturedImage>,
    frame_id: i32,
    last_frame_time: Instant,
}

impl SimCamera {
    pub async fn new(controller: SimController) -> Result<Self, CanonicalError> {
        let (key, frame) = {
            let state = controller.state.lock().unwrap();
            ((state.generation, state.index), state.frames[state.index].clone())
        };
        let path = frame.path.clone();
        let image = tokio::task::spawn_blocking(move || decode(&path))
            .await.unwrap()
            .map_err(|e| failed_precondition_error(&format!(
                "Cannot decode {:?}: {}", frame.path, e)))?;
        let binned_image = bin_2x2(&image);
        let mut sc = SimCamera{
            controller,
            exposure_duration: Duration::from_millis(50),
            offset: Offset::new(3),
            gain: Gain::new(50),
            update_interval: Duration::ZERO,
            current: DecodedFrame{key, image: Arc::new(image),
                                  binned_image: Arc::new(binned_image),
                                  exposure: frame.exposure},
            prefetch: None,
            most_recent_capture: None,
            frame_id: 0,
            last_frame_time: Instant::now(),
        };
        sc.emit_frame().await;
        Ok(sc)
    }

    pub fn controller(&self) -> SimController {
        self.controller.clone()
    }

    // Time between frames: the frame's recorded exposure (else our nominal
    // exposure), no faster than the update interval, scaled by playback speed.
    fn frame_interval(&self) -> Duration {
        let speed = self.controller.state.lock().unwrap().speed;
        let exposure = self.current.exposure.unwrap_or(self.exposure_duration);
        cmp::max(exposure, self.update_interval).div_f64(speed)
    }

    // Emits the frame at the controller's current index, then advances the
    // index if playing.
    async fn emit_frame(&mut self) {
        let (key, frame) = {
            let state = self.controller.state.lock().unwrap();
            ((state.generation, state.index), state.frames[state.index].clone())
        };
        if key != self.current.key {
            self.load_frame(key, &frame).await;
        }
        self.most_recent_capture = Some(CapturedImage {
            capture_params: CaptureParams {
                exposure_duration: self.current.exposure
                    .unwrap_or(self.exposure_duration),
                gain: self.gain,
                offset: self.offset,
            },
            params_accurate: true,
            image: self.current.image.clone(),
            binned_image: self.current.binned_image.clone(),
            is_color: false,
            readout_time: SystemTime::now(),
            readout_instant: Instant::now(),
            processing_duration: None,
        });
        self.frame_id += 1;
        self.last_frame_time = Instant::now();

        let next = {
            let mut state = self.controller.state.lock().unwrap();
            state.frame_id = self.frame_id;
            // Only advance if nobody moved the index meanwhile.
            if state.playing && (state.generation, state.index) == key {
                if state.index + 1 < state.frames.len() {
                    state.index += 1;
                } else if state.looping {
                    state.index = 0;
                } else {
                    state.playing = false;
                }
            }
            ((state.generation, state.index), state.frames[state.index].clone())
        };
        self.start_prefetch(next.0, next.1);
    }

    fn start_prefetch(&mut self, key: FrameKey, frame: SimFrame) {
        if key == self.current.key {
            return;
        }
        if let Some((pending_key, _)) = &self.prefetch {
            if *pending_key == key {
                return;
            }
        }
        if let Some((_, handle)) = self.prefetch.take() {
            handle.abort();
        }
        self.prefetch = Some((key, tokio::task::spawn_blocking(
            move || decode(&frame.path))));
    }

    // Makes `frame` current, from the prefetch if it has it. On a decode
    // failure the previous image keeps being emitted.
    async fn load_frame(&mut self, key: FrameKey, frame: &SimFrame) {
        let decoded = match self.prefetch.take() {
            Some((pending_key, handle)) if pending_key == key => {
                handle.await.unwrap_or_else(|e| Err(e.to_string()))
            }
            other => {
                if let Some((_, handle)) = other {
                    handle.abort();
                }
                let path = frame.path.clone();
                tokio::task::spawn_blocking(move || decode(&path)).await
                    .unwrap_or_else(|e| Err(e.to_string()))
            }
        };
        let image = match decoded {
            Ok(image) => image,
            Err(e) => {
                warn!("Cannot decode {:?}, repeating previous frame: {}",
                      frame.path, e);
                self.current.key = key;  // Don't retry every capture.
                return;
            }
        };
        // Dimensions were checked from headers; guard against a file that
        // changed since.
        if image.dimensions() != self.current.image.dimensions() {
            warn!("{:?} is {:?}, expected {:?}; repeating previous frame",
                  frame.path, image.dimensions(), self.current.image.dimensions());
            self.current.key = key;
            return;
        }
        let binned_image = bin_2x2(&image);
        self.current = DecodedFrame{key, image: Arc::new(image),
                                    binned_image: Arc::new(binned_image),
                                    exposure: frame.exposure};
    }
}

#[async_trait]
impl AbstractCamera for SimCamera {
    async fn model(&self) -> String {
        "SimCamera".to_string()
    }

    async fn model_detail(&self) -> Option<String> {
        Some(self.controller.status().source)
    }

    async fn dimensions(&self) -> (u32, u32) {
        self.current.image.dimensions()
    }

    fn is_color(&self) -> bool {
        false
    }

    async fn sensor_size(&self) -> (f32, f32) {
        (4.8, 3.6)
    }

    async fn optimal_gain(&self) -> Gain {
        Gain::new(50)
    }

    async fn set_exposure_duration(&mut self, _exp_duration: Duration)
                                   -> Result<(), CanonicalError> {
        Ok(())  // Quietly ignore.
    }
    async fn get_exposure_duration(&self) -> Duration {
        self.exposure_duration
    }

    async fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
        self.gain = gain;
        Ok(())
    }
    async fn get_gain(&self) -> Gain {
        self.gain
    }

    async fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
        self.offset = offset;
        Ok(())
    }
    async fn get_offset(&self) -> Offset {
        self.offset
    }

    async fn set_update_interval(&mut self, update_interval: Duration)
                                 -> Result<(), CanonicalError> {
        self.update_interval = update_interval;
        Ok(())
    }

    async fn capture_image(&mut self, prev_frame_id: Option<i32>)
                           -> Result<(CapturedImage, i32), CanonicalError> {
        let need_new_image =
            prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id;
        let next_frame_time = self.last_frame_time + self.frame_interval();
        let sleep_interval = next_frame_time.saturating_duration_since(Instant::now());
        if sleep_interval == Duration::ZERO {
            self.emit_frame().await;
        } else if need_new_image {
            tokio::time::sleep(sleep_interval).await;
            self.emit_frame().await;
        }
        Ok((self.most_recent_capture.clone().unwrap(), self.frame_id))
    }

    async fn try_capture_image(
        &mut self, prev_frame_id: Option<i32>)
        -> Result<Option<(CapturedImage, i32)>, CanonicalError>
    {
        let need_new_image =
            prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id;
        let next_frame_time = self.last_frame_time + self.frame_interval();
        let sleep_interval = next_frame_time.saturating_duration_since(Instant::now());
        if sleep_interval == Duration::ZERO {
            self.emit_frame().await;
        } else if need_new_image {
            return Ok(None);
        }
        Ok(Some((self.most_recent_capture.clone().unwrap(), self.frame_id)))
    }

    async fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration> {
        if prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id {
            let next_frame_time = self.last_frame_time + self.frame_interval();
            let now = Instant::now();
            if now >= next_frame_time {
                return Some(Duration::ZERO);
            }
            Some(next_frame_time - now)
        } else {
            Some(Duration::ZERO)
        }
    }

    async fn start(&mut self) -> Result<(), CanonicalError> {
        Ok(())
    }

    async fn stop(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::Luma;

    // Writes `n` w x h PNGs whose pixel (0,0) holds the frame number.
    fn write_frames(dir: &Path, names: &[&str], w: u32, h: u32) {
        for (i, name) in names.iter().enumerate() {
            let mut img = GrayImage::new(w, h);
            img.put_pixel(0, 0, Luma([i as u8 + 1]));
            img.save(dir.join(name)).unwrap();
        }
    }

    fn temp_dir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "sim_camera_test_{}_{}", tag, std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn marker(c: &CapturedImage) -> u8 {
        c.image.get_pixel(0, 0).0[0]
    }

    async fn next(cam: &mut SimCamera, prev: i32) -> (CapturedImage, i32) {
        cam.capture_image(Some(prev)).await.unwrap()
    }

    #[test]
    fn exposure_parsed_from_filename() {
        assert_eq!(exposure_from_filename(
            Path::new("img_000003_70ms_20260722_055948.png")),
                   Some(Duration::from_millis(70)));
        assert_eq!(exposure_from_filename(Path::new("bright_star.jpg")), None);
    }

    #[tokio::test]
    async fn plays_directory_in_name_order_and_loops() {
        let dir = temp_dir("order");
        write_frames(&dir, &["b_1ms.png", "c_1ms.png", "a_1ms.png"], 8, 6);
        let ctl = SimController::new(&dir, Path::new(".")).unwrap();
        // Sorted: a(3), b(1), c(2).
        let mut cam = SimCamera::new(ctl.clone()).await.unwrap();
        assert_eq!(cam.dimensions().await, (8, 6));
        let (c, mut id) = cam.capture_image(None).await.unwrap();
        assert_eq!(marker(&c), 3);
        // Paused: same frame content, but new frame ids.
        let (c, id2) = next(&mut cam, id).await;
        assert_eq!(marker(&c), 3);
        assert!(id2 > id);
        id = id2;

        ctl.play();
        let mut seen = vec![];
        for _ in 0..4 {
            let (c, i) = next(&mut cam, id).await;
            seen.push(marker(&c));
            id = i;
        }
        // The frame current when play() was called is emitted once more,
        // then playback advances and wraps.
        assert_eq!(seen, vec![3, 1, 2, 3]);
        assert_eq!(ctl.status().frame_id, id);
    }

    #[tokio::test]
    async fn manifest_sets_order_and_exposure() {
        let dir = temp_dir("manifest");
        write_frames(&dir, &["x.png", "y.png"], 4, 4);
        std::fs::write(dir.join("manifest.csv"),
                       "filename,exposure_ms\ny.png,7\nx.png,9\n").unwrap();
        let ctl = SimController::new(&dir, Path::new(".")).unwrap();
        let mut cam = SimCamera::new(ctl.clone()).await.unwrap();
        let (c, id) = cam.capture_image(None).await.unwrap();
        assert_eq!(marker(&c), 2);
        assert_eq!(c.capture_params.exposure_duration, Duration::from_millis(7));
        ctl.step(1);
        let (c, _) = next(&mut cam, id).await;
        assert_eq!(marker(&c), 1);
        assert_eq!(c.capture_params.exposure_duration, Duration::from_millis(9));
    }

    #[tokio::test]
    async fn step_seek_and_stop_at_end() {
        let dir = temp_dir("seek");
        write_frames(&dir, &["0_1ms.png", "1_1ms.png", "2_1ms.png"], 4, 4);
        let ctl = SimController::new(&dir, Path::new(".")).unwrap();
        let mut cam = SimCamera::new(ctl.clone()).await.unwrap();
        assert_eq!(ctl.step(-1).index, 2);
        assert_eq!(ctl.step(2).index, 1);
        assert!(ctl.seek(3).is_err());
        ctl.seek(1).unwrap();
        ctl.set_looping(false);
        ctl.play();
        let (_, mut id) = cam.capture_image(None).await.unwrap();
        let mut seen = vec![];
        for _ in 0..3 {
            let (c, i) = next(&mut cam, id).await;
            seen.push(marker(&c));
            id = i;
        }
        // Frames 1 and 2, then holds on the last frame.
        assert_eq!(seen, vec![2, 3, 3]);
        assert!(!ctl.status().playing);
        // Play from the end restarts at the beginning.
        assert_eq!(ctl.play().index, 0);
    }

    #[tokio::test]
    async fn set_source_switches_frames_and_checks_size() {
        let dir = temp_dir("source");
        std::fs::create_dir_all(dir.join("big")).unwrap();
        write_frames(&dir, &["one.png"], 4, 4);
        write_frames(&dir.join("big"), &["big.png"], 8, 8);
        std::fs::create_dir_all(dir.join("two")).unwrap();
        write_frames(&dir.join("two"), &["p.png", "q.png"], 4, 4);

        let ctl = SimController::new(&dir, Path::new("one.png")).unwrap();
        let mut cam = SimCamera::new(ctl.clone()).await.unwrap();
        let (_, id) = cam.capture_image(None).await.unwrap();

        assert!(ctl.set_source(Path::new("big"), None).is_err());
        assert!(ctl.set_source(Path::new(".."), None).is_err());
        assert!(ctl.set_source(Path::new("/etc"), None).is_err());
        assert!(ctl.set_source(Path::new("two"), Some(5)).is_err());
        assert_eq!(ctl.status().source, "one.png");

        let status = ctl.set_source(Path::new("two"), Some(1)).unwrap();
        assert_eq!((status.count, status.index), (2, 1));
        assert_eq!(status.frame_name, "q.png");
        let (c, _) = next(&mut cam, id).await;
        assert_eq!(marker(&c), 2);
    }

    #[tokio::test]
    async fn speed_scales_interval() {
        let dir = temp_dir("speed");
        write_frames(&dir, &["f_100ms.png"], 4, 4);
        let ctl = SimController::new(&dir, Path::new("f_100ms.png")).unwrap();
        let mut cam = SimCamera::new(ctl.clone()).await.unwrap();
        assert_eq!(cam.frame_interval(), Duration::from_millis(100));
        ctl.set_speed(4.0).unwrap();
        assert_eq!(cam.frame_interval(), Duration::from_millis(25));
        assert!(ctl.set_speed(0.0).is_err());
        cam.set_update_interval(Duration::from_millis(400)).await.unwrap();
        assert_eq!(cam.frame_interval(), Duration::from_millis(100));
    }
}
