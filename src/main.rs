use tello::Drone;

use image::{DynamicImage, GenericImageView, ImageFormat};
use rustface::{Detector, ImageData};

fn detect_faces(detector: &mut Box<dyn Detector>, img: DynamicImage) -> (i32, i32) {
    // ".to_luma8()" verandert het plaatje in zwart en wit anders kan het niet worden gelezen door het detectie model.
    let grayed = img.to_luma8();
    let (width, height) = img.dimensions();
    
    // De detector maakt niet gebruik van Image object, maar van een ImageData object, dus die zet ik hier om.
    let mut image = ImageData::new(&grayed, width, height);
    let faces = detector.detect(&mut image);
    
    // Geen gezichten? Geef (0, 0) terug.
    if faces.len() == 0 {
        return (0, 0);
    }
    
    // Ik ga over alle gevonden gezichten en pak alleen de x,y coordinaten en zet die in een lijstje
    let x_coords: Vec<i32> = faces.iter().map(|face| face.bbox().x()).collect();
    let y_coords: Vec<i32> = faces.iter().map(|face| face.bbox().y()).collect();
    
    // Ik tel alle x, y coordinaten bij elkaar op en deel deze door de hoeveelheid gezichten, hierdoor krijg ik de gemiddelde positie van alle gezichten. 
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
    // Ik kan niet de drone 100% laten draaien, want dan draait ie hoogstwaarschijnlijk zo ver dat de gezichten uit frame gaan, dus pak ik maar de helft.
    // De drone gebruikt 0-3600 in plaats van 0-360 graden, dus daarom doe ik het keer 10.
    let rotation = (0.05 * delta_x as f32) as i32 * 10;

    if rotation <= 0 {
        return (Direction::CounterClockWise, -rotation);
    }
    (Direction::ClockWise, rotation)
}

#[tokio::main]
async fn main() {
    const CAM_DIMENSIONS: (i32, i32) = (1280, 720);
    
    // Hier laad ik de SeetaFaceEngine
    let mut detector = rustface::create_detector("./seeta_fd_frontal_v1.0.bin").unwrap();
    // Ik zet dit op twintig, want anders heb je meer kans dat er gezichten worden gedetecteerd die er helemaal niet zijn.
    // Doordat ik dit op 20 zet moeten de gezichten minimaal 20px*20px groot zijn.
    detector.set_min_face_size(20);
    
    // De rest is gelaten op wat de documentatie zei.     
    detector.set_score_thresh(2.0);
    detector.set_pyramid_scale_factor(0.8);
    detector.set_slide_window_step(15, 15);
    
    // Hier wordt de verbinding met de drone gemaakt.
    let mut drone = Drone::new("192.168.10.1:8889").command_mode();
    // De drone duurt altijd even om op de starten, dus hier wordt een leeg commando gestuurd. 
    drone.enable().await.unwrap();
    drone.take_off().await.unwrap();
    drone.video_on().await.unwrap();

    let mut video = drone.video_receiver().unwrap();

    loop {
        let image_res = match video.blocking_recv() {
            // We hebben wel wat ontvangen, we gaan verder.
            Some(res) => res,
            // We hebben geen foto van de drone ontvangen, laten we opnieuw proberen. 
            None => {
                continue;
            }
        };
        // Ik krijg van de Tello drone niet een bruikbaar plaatje terug, dus moet ik het omzetten naar een type waar de computer wat mee kan.
        // Gelukkig heeft de Image library hier een standaard functie voor... 
        let target_coords = match image::load_from_memory_with_format(&image_res, ImageFormat::Png)
        {
            Ok(img) => detect_faces(&mut detector, img),
            // Als er iets mis is met het plaatje van de Tello drone kan de library het niet goed laden en wordt deze functie aangeroepen.
            Err(_) => {
                println!("Failed to load image from memory");
                continue;
            }
        };

        // Als er geen gezichten worden gevonden krijgen ik (0, 0) terug, dus begin ik de functie gewoon weer opnieuw.
        if target_coords == (0, 0) {
            continue;
        }
        
        // Ik vergelijk de gemiddelde positie van de gezichten met het middelpunt van de X-as. Deze stop ik in de functie "calc_rotation()"
        let (direction, rotation) = calc_rotation(CAM_DIMENSIONS.0 / 2 - target_coords.0);

        if direction == Direction::ClockWise {
            // Draait de drone met de klok mee.
            drone.cw(rotation as u32).await.unwrap();
        } else {
            // Draait de drone tegen de klok in. 
            drone.ccw(rotation as u32).await.unwrap();
        }
    }
}
