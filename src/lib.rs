use std::process;
use wasm_bindgen::prelude::*;

const VISION_RANGE: f32 = 60.0;
const CONVERT_RANGE: f32 = 10.0;

const MAX_SPEED: f32 = 1.0;

/*
const SEPERATION_FORCE: f32 = 8.0;
const COHESION_FORCE: f32 = 0.05;
const ALIGN_FORCE: f32 = 0.2;

const PREY_FORCE: f32 = 0.05;
const PRED_FORCE: f32 = 0.05;
*/

const WIDTH: f32 = 1620.0;
const HEIGHT: f32 = 1080.0;

const GRID_ROWS: usize = (WIDTH / VISION_RANGE) as usize;
const GRID_COLS: usize = (HEIGHT / VISION_RANGE) as usize;

const GRID_CNT: usize = GRID_ROWS * GRID_COLS;

#[inline]
pub fn unwrap_abort<T>(option: Option<T>) -> T {
    match option {
        Some(t) => t,
        None => process::abort(),
    }
}

#[derive(Copy, Clone)]
struct Vector2 {
    x: f32,
    y: f32,
}

impl Vector2 {
    fn new(x: f32, y: f32) -> Self {
        Vector2 { x, y }
    }

    fn sub(&self, other: Vector2) -> Vector2 {
        Vector2::new(self.x - other.x, self.y - other.y)
    }

    fn mul(&self, scalar: f32) -> Vector2 {
        Vector2::new(scalar * self.x, scalar * self.y)
    }

    fn add_mut(&mut self, other: Vector2) {
        self.x += other.x;
        self.y += other.y;
    }

    fn sub_mut(&mut self, other: Vector2) {
        self.x -= other.x;
        self.y -= other.y;
    }

    fn mul_mut(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
    }

    fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    fn distance_to(&self, other: &Vector2) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

struct Boid {
    hand: u8,
    hash: usize,
    position: Vector2,
    velocity: Vector2,
}

impl Boid {
    fn new(hand: u8, position: Vector2, velocity: Vector2) -> Self {
        let mut boid = Boid {
            hand,
            hash: 0,
            position,
            velocity,
        };
        boid.rehash();
        boid
    }

    fn rehash(&mut self) {
        let x = (self.position.x / VISION_RANGE) as usize;
        let y = (self.position.y / VISION_RANGE) as usize;

        self.hash = x + GRID_ROWS * y;
    }

    fn update(&mut self, acceleration: Vector2) {
        self.position.x += self.velocity.x;
        self.position.y += self.velocity.y;

        self.position.x = self.position.x.rem_euclid(WIDTH);
        self.position.y = self.position.y.rem_euclid(HEIGHT);

        if self.position.x == WIDTH {
            self.position.x -= 1e3;
        }

        if self.position.y == HEIGHT {
            self.position.y -= 1e3;
        }

        self.rehash();

        self.velocity.add_mut(acceleration);

        let speed = self.velocity.length();

        if speed > MAX_SPEED {
            self.velocity.mul_mut(MAX_SPEED / speed);
        }
    }
}

#[wasm_bindgen]
pub struct Flock {
    count: usize,
    boids: Vec<Boid>,
    table: Vec<Vec<usize>>,
}

#[wasm_bindgen]
impl Flock {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Flock {
            count: 0,
            boids: Vec::new(),
            table: vec![Vec::new(); GRID_CNT],
        }
    }

    pub fn add_boid(
        &mut self,
        hand: u8,
        position_x: f32,
        position_y: f32,
        velocity_x: f32,
        velocity_y: f32,
    ) {
        let boid = Boid::new(
            hand,
            Vector2::new(position_x * WIDTH, position_y * HEIGHT),
            Vector2::new(velocity_x, velocity_y),
        );
        unwrap_abort(self.table.get_mut(boid.hash)).push(self.count);
        self.count += 1;
        self.boids.push(boid);
    }

    pub fn render(&self) {
        for boid in &self.boids {
            draw(boid.hand, boid.position.x, boid.position.y);
        }
    }

    pub fn update(&mut self, seperation_force: f32, cohesion_force: f32, align_force: f32, pred_force: f32, prey_force: f32) {
        for i in 0..self.count {
            let mut acceleration = Vector2::new(0.0, 0.0);

            let mut average_position = Vector2::new(0.0, 0.0);
            let mut average_velocity = Vector2::new(0.0, 0.0);

            let mut prey_average_position = Vector2::new(0.0, 0.0);
            let mut pred_average_position = Vector2::new(0.0, 0.0);

            let mut neighbor_count = 0;

            let mut prey_count = 0;
            let mut pred_count = 0;

            let mut should_change = false;
            let mut should_not_change = false;

            let hash = unwrap_abort(self.boids.get(i)).hash;

            let mut iterator_vec = vec![unwrap_abort(self.table.get(hash)).iter()];

            let bool1 = hash % GRID_ROWS == 0;
            let bool2 = (hash + 1) % GRID_ROWS == 0;
            let bool3 = hash < GRID_ROWS;
            let bool4 = ((hash + GRID_ROWS) % GRID_CNT) < GRID_ROWS;

            if !bool1 {
                iterator_vec.push(unwrap_abort(self.table.get((hash - 1) % GRID_CNT)).iter());
            }

            if !bool2 {
                iterator_vec.push(unwrap_abort(self.table.get((hash + 1) % GRID_CNT)).iter());
            }

            if !bool3 {
                iterator_vec.push(unwrap_abort(self.table.get((hash - GRID_ROWS) % GRID_CNT)).iter());
            }

            if !bool4 {
                iterator_vec.push(unwrap_abort(self.table.get((hash + GRID_ROWS) % GRID_CNT)).iter());
            }

            if !(bool1 && bool3) {
                iterator_vec.push(unwrap_abort(self.table.get((hash - GRID_ROWS - 1) % GRID_CNT)).iter());
            }

            if !(bool1 && bool4) {
                iterator_vec.push(unwrap_abort(self.table.get((hash + GRID_ROWS - 1) % GRID_CNT)).iter());
            }

            if !(bool2 && bool3) {
                iterator_vec.push(unwrap_abort(self.table.get((hash - GRID_ROWS + 1) % GRID_CNT)).iter());
            }

            if !(bool2 && bool4) {
                iterator_vec.push(unwrap_abort(self.table.get((hash + GRID_ROWS + 1) % GRID_CNT)).iter());
            }

            /*
            let iterator = unwrap_abort(self.table.get(hash))
                .iter()
                .chain(unwrap_abort(self.table.get((hash - 1) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash + 1) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash - GRID_ROWS) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash + GRID_ROWS) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash - GRID_ROWS - 1) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash + GRID_ROWS - 1) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash - GRID_ROWS + 1) % GRID_CNT)).iter())
                .chain(unwrap_abort(self.table.get((hash + GRID_ROWS + 1) % GRID_CNT)).iter());
            */
            
            let iterator = iterator_vec.iter().flat_map(|it| it.clone());

            for j in iterator {
                if i == *j {
                    continue;
                }

                let boid = &unwrap_abort(self.boids.get(i));
                let other = &unwrap_abort(self.boids.get(*j));

                let distance = boid.position.distance_to(&other.position);

                if distance >= VISION_RANGE {
                    continue;
                }

                if boid.hand == other.hand {
                    let difference = boid.position.sub(other.position);

                    acceleration
                        .add_mut(difference.mul(seperation_force / difference.length_squared()));

                    average_position.add_mut(other.position);
                    average_velocity.add_mut(other.velocity);
                    neighbor_count += 1;
                }

                if boid.hand == ((other.hand + 1) % 3) {
                    should_change = (distance < CONVERT_RANGE) | should_change;
                    pred_average_position.add_mut(other.position);
                    pred_count += 1;
                }

                if boid.hand == ((other.hand + 2) % 3) {
                    should_not_change = (distance < CONVERT_RANGE) | should_not_change;
                    prey_average_position.add_mut(other.position);
                    prey_count += 1;
                }
            }

            if neighbor_count > 0 {
                average_position.mul_mut(1.0 / neighbor_count as f32);
                average_position.sub_mut(unwrap_abort(self.boids.get(i)).position);
                average_position.mul_mut(cohesion_force);

                average_velocity.mul_mut(align_force / neighbor_count as f32);

                acceleration.add_mut(average_position);
                acceleration.add_mut(average_velocity);
            }

            if pred_count > 0 {
                pred_average_position.mul_mut(1.0 / pred_count as f32);
                pred_average_position.sub_mut(unwrap_abort(self.boids.get(i)).position);
                pred_average_position.mul_mut(-pred_force);

                acceleration.add_mut(pred_average_position);
            }

            if prey_count > 0 {
                prey_average_position.mul_mut(1.0 / prey_count as f32);
                prey_average_position.sub_mut(unwrap_abort(self.boids.get(i)).position);
                prey_average_position.mul_mut(prey_force);

                acceleration.add_mut(prey_average_position);
            }

            unwrap_abort(self.boids.get_mut(i)).update(acceleration);

            if should_change && !should_not_change {
                unwrap_abort(self.boids.get_mut(i)).hand =
                    (unwrap_abort(self.boids.get(i)).hand + 2) % 3;
            }
        }

        self.table.iter_mut().for_each(|entry| entry.clear());
        self.boids
            .iter()
            .enumerate()
            .for_each(|(index, boid)| unwrap_abort(self.table.get_mut(boid.hash)).push(index));
    }
}

#[wasm_bindgen]
extern "C" {
    fn draw(c: u8, x: f32, y: f32);
}
