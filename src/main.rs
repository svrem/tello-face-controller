use tello::Drone;

use image::{DynamicImage, GenericImageView, ImageFormat};
use rustface::{Detector, ImageData};

fn detect_faces(detector: &mut Box<dyn Detector>, img: DynamicImage) -> (i32, i32) {
    let grayed = img.to_luma8();
    let (width, height) = img.dimensions();

    let mut image = ImageData::new(&grayed, width, height);
    let faces = detector.detect(&mut image);

    if faces.len() == 0 {
        return (0, 0);
    }

    let x_coords: Vec<i32> = faces.iter().map(|face| face.bbox().x()).collect();
    let y_coords: Vec<i32> = faces.iter().map(|face| face.bbox().y()).collect();

    let target_x: i32 = x_coords.iter().sum::<i32>() / faces.len() as i32;
    let target_y: i32 = y_coords.iter().sum::<i32>() / faces.len() as i32;

    return (target_x, target_y);
}

#[derive(PartialEq)]
enum Direction {
    ClockWise,
    CounterClockWise,
}

fn calc_rotation(delta_x: i32) -> (Direction, i32) {
    // times 10 because the tello module thinks it's special (they set their range to 0-3600)
    let rotation = (0.05 * delta_x as f32) as i32 * 10;

    if rotation <= 0 {
        return (Direction::CounterClockWise, -rotation);
    }
    (Direction::ClockWise, rotation)
}

#[tokio::main]
async fn main() {
    const CAM_DIMENSIONS: (i32, i32) = (1280, 720);

    let mut detector = rustface::create_detector("./seeta_fd_frontal_v1.0.bin").unwrap();
    detector.set_min_face_size(20);
    detector.set_score_thresh(2.0);
    detector.set_pyramid_scale_factor(0.8);
    detector.set_slide_window_step(15, 15);

    let mut drone = Drone::new("192.168.10.1:8889").command_mode();
    drone.enable().await.unwrap();
    drone.take_off().await.unwrap();
    drone.video_on().await.unwrap();

    let mut video = drone.video_receiver().unwrap();

    loop {
        let image_res = match video.blocking_recv() {
            Some(res) => res,
            None => {
                continue;
            }
        };

        let target_coords = match image::load_from_memory_with_format(&image_res, ImageFormat::Png)
        {
            Ok(img) => detect_faces(&mut detector, img),
            Err(_) => {
                println!("Failed to load image from memory");
                continue;
            }
        };

        if target_coords == (0, 0) {
            continue;
        }

        let (direction, rotation) = calc_rotation(CAM_DIMENSIONS.0 / 2 - target_coords.0);

        if direction == Direction::ClockWise {
            drone.cw(rotation as u32).await.unwrap();
        } else {
            drone.ccw(rotation as u32).await.unwrap();
        }
    }
}
