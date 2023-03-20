use std::process;
use core::arch::wasm32::*;
use wasm_bindgen::prelude::*;

const VISION_RANGE: f32 = 60.0;
const CONVERT_RANGE: f32 = 10.0;

const MAX_SPEED: f32 = 10.0;

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

struct Boid {
    hand: u32,
    hash: usize,
    position_x: f32,
    position_y: f32,
    velocity_x: f32,
    velocity_y: f32,
}

impl Boid {
    fn new(hand: u32, position_x: f32, position_y: f32, velocity_x: f32, velocity_y: f32) -> Self {
        let mut boid = Boid {
            hand,
            hash: 0,
            position_x,
            position_y,
            velocity_x,
            velocity_y,
        };

        boid.rehash();

        boid
    }

    fn rehash(&mut self) {
        self.hash = (self.position_x / VISION_RANGE) as usize + GRID_ROWS * (self.position_y / VISION_RANGE) as usize;
    }

    fn update(&mut self, acceleration_x: f32, acceleration_y: f32) {
        self.position_x += self.velocity_x;
        self.position_y += self.velocity_y;

        self.position_x = self.position_x.rem_euclid(WIDTH);
        self.position_y = self.position_y.rem_euclid(HEIGHT);

        if self.position_x == WIDTH {
            self.position_x -= 1e3;
        }

        if self.position_y == HEIGHT {
            self.position_y -= 1e3;
        }

        self.rehash();

        self.velocity_x += acceleration_x;
        self.velocity_y += acceleration_y;

        let speed = (self.velocity_x * self.velocity_x + self.velocity_y * self.velocity_y).sqrt();

        if speed > MAX_SPEED {
            self.velocity_x *= MAX_SPEED / speed;
            self.velocity_y *= MAX_SPEED / speed;
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
        hand: u32,
        position_x: f32,
        position_y: f32,
        velocity_x: f32,
        velocity_y: f32,
    ) {
        let boid = Boid::new(
            hand,
            position_x,
            position_y,
            velocity_x,
            velocity_y,
        );

        unwrap_abort(self.table.get_mut(boid.hash)).push(self.count);

        self.count += 1;
        self.boids.push(boid);
    }

    pub fn render(&self) {
        for boid in &self.boids {
            draw(boid.hand, boid.position_x, boid.position_y);
        }
    }

    #[target_feature(enable = "simd128")]
    pub fn update(&mut self, seperation_weight_single: f32, cohesion: f32, align: f32, pred: f32, prey: f32) {
        let seperation_weight = f32x4(seperation_weight_single, seperation_weight_single, seperation_weight_single, seperation_weight_single);
        let cohesion_weight = f32x4(cohesion, cohesion, cohesion, cohesion);

        let vision_range = f32x4(VISION_RANGE, VISION_RANGE, VISION_RANGE, VISION_RANGE);

        let one = u32x4(1, 1, 1, 1);
        let eps = f32x4(1e-8, 1e-8, 1e-8, 1e-8);

        let mut accelerations_x = vec![0.0; self.boids.len()];
        let mut accelerations_y = vec![0.0; self.boids.len()];

        for hash in 0..GRID_CNT {
            let current_grid = unwrap_abort(self.table.get(hash)).clone();

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

            let main_iterator = current_grid.chunks_exact(4);

            let iterator = iterator_vec.iter().flat_map(|it| it.clone());

            for i in main_iterator.clone() {
                let mut acceleration_x = f32x4(0.0, 0.0, 0.0, 0.0);
                let mut acceleration_y = f32x4(0.0, 0.0, 0.0, 0.0);

                let mut average_position_x = f32x4(0.0, 0.0, 0.0, 0.0);
                let mut average_position_y = f32x4(0.0, 0.0, 0.0, 0.0);

                let mut average_velocity_x = f32x4(0.0, 0.0, 0.0, 0.0);
                let mut average_velocity_y = f32x4(0.0, 0.0, 0.0, 0.0);

                let mut neighbor_count = u32x4(0, 0, 0, 0);

                let boid0 = &unwrap_abort(self.boids.get(*unwrap_abort(i.get(0))));
                let boid1 = &unwrap_abort(self.boids.get(*unwrap_abort(i.get(1))));
                let boid2 = &unwrap_abort(self.boids.get(*unwrap_abort(i.get(2))));
                let boid3 = &unwrap_abort(self.boids.get(*unwrap_abort(i.get(3))));

                let position_x = f32x4(boid0.position_x, boid1.position_x, boid2.position_x, boid3.position_x);
                let position_y = f32x4(boid0.position_y, boid1.position_y, boid2.position_y, boid3.position_y);

                for j in iterator.clone() {
                    let other = &unwrap_abort(self.boids.get(*j));

                    let other_position_x = f32x4(other.position_x, other.position_x, other.position_x, other.position_x);
                    let other_position_y = f32x4(other.position_y, other.position_y, other.position_y, other.position_y);

                    let other_velocity_x = f32x4(other.velocity_x, other.velocity_x, other.velocity_x, other.velocity_x);
                    let other_velocity_y = f32x4(other.velocity_y, other.velocity_y, other.velocity_y, other.velocity_y);

                    let is_same_hand = u32x4((boid0.hand == other.hand) as u32, (boid1.hand == other.hand) as u32, (boid2.hand == other.hand) as u32, (boid3.hand == other.hand) as u32);
                    //let is_next_hand = u32x4((self0.hand == ((other0.hand + 1) % 3)) as u32, (self1.hand == ((other1.hand + 1) % 3)) as u32, (self2.hand == ((other2.hand + 1) % 3)) as u32, (self3.hand == ((other3.hand + 1) % 3)) as u32);
                    //let is_prev_hand = u32x4((self0.hand == ((other0.hand + 2) % 3)) as u32, (self1.hand == ((other1.hand + 2) % 3)) as u32, (self2.hand == ((other2.hand + 2) % 3)) as u32, (self3.hand == ((other3.hand + 2) % 3)) as u32);

                    let diff_x = f32x4_sub(position_x, other_position_x);
                    let diff_y = f32x4_sub(position_y, other_position_y);

                    let distance_squared = f32x4_add(f32x4_add(f32x4_mul(diff_x, diff_x), f32x4_mul(diff_y, diff_y)), eps);
                    let distance = f32x4_sqrt(distance_squared);

                    let is_in_range = f32x4_lt(distance, vision_range);

                    let is_same_hand = u32x4_mul(is_same_hand, is_in_range);
                    //let is_next_hand = u32x4_mul(is_next_hand, is_in_range);
                    //let is_prev_hand = u32x4_mul(is_prev_hand, is_in_range);

                    let seperation_relative_weight = f32x4_div(seperation_weight, distance);

                    //log(f32x4_extract_lane::<0>(seperation_weight));

                    acceleration_x = f32x4_add(acceleration_x, f32x4_mul(diff_x, seperation_relative_weight));
                    acceleration_y = f32x4_add(acceleration_y, f32x4_mul(diff_y, seperation_relative_weight));

                    average_position_x = f32x4_add(average_position_x, v128_and(other_position_x, is_same_hand));
                    average_position_y = f32x4_add(average_position_y, v128_and(other_position_y, is_same_hand));

                    average_velocity_x = f32x4_add(average_velocity_x, v128_and(other_velocity_x, is_same_hand));
                    average_velocity_y = f32x4_add(average_velocity_y, v128_and(other_velocity_y, is_same_hand));

                    neighbor_count = u32x4_add(neighbor_count, u32x4_mul(is_same_hand, one));
                }

                /*
                let neighbor_count = f32x4_convert_u32x4(neighbor_count);

                average_position_x = f32x4_div(average_position_x, neighbor_count);
                average_position_y = f32x4_div(average_position_y, neighbor_count);

                average_position_x = f32x4_sub(average_position_x, position_x);
                average_position_y = f32x4_sub(average_position_y, position_y);

                average_position_x = f32x4_mul(average_position_x, cohesion_weight);
                average_position_y = f32x4_mul(average_position_y, cohesion_weight);

                acceleration_x = f32x4_add(acceleration_x, average_position_x);
                acceleration_y = f32x4_add(acceleration_y, average_position_y);
                */

                accelerations_x[*unwrap_abort(i.get(0))] = f32x4_extract_lane::<0>(acceleration_x);
                accelerations_x[*unwrap_abort(i.get(1))] = f32x4_extract_lane::<1>(acceleration_x);
                accelerations_x[*unwrap_abort(i.get(2))] = f32x4_extract_lane::<2>(acceleration_x);
                accelerations_x[*unwrap_abort(i.get(3))] = f32x4_extract_lane::<3>(acceleration_x);

                accelerations_y[*unwrap_abort(i.get(0))] = f32x4_extract_lane::<0>(acceleration_y);
                accelerations_y[*unwrap_abort(i.get(1))] = f32x4_extract_lane::<1>(acceleration_y);
                accelerations_y[*unwrap_abort(i.get(2))] = f32x4_extract_lane::<2>(acceleration_y);
                accelerations_y[*unwrap_abort(i.get(3))] = f32x4_extract_lane::<3>(acceleration_y);

            }

            for i in main_iterator.remainder() {
                let mut acceleration_x = 0.0;
                let mut acceleration_y = 0.0;

                let mut average_position_x = 0.0;
                let mut average_position_y = 0.0;

                let mut average_velocity_x = 0.0;
                let mut average_velocity_y = 0.0;

                let mut neighbor_count = 0;

                let boid = &unwrap_abort(self.boids.get(*i));

                for j in iterator.clone() {
                    if i == j {
                        continue;
                    }

                    let other = &unwrap_abort(self.boids.get(*j));

                    let diff_x = boid.position_x - other.position_x;
                    let diff_y = boid.position_y - other.position_y;

                    let distance_squared = diff_x * diff_x + diff_y * diff_y;
                    let distance = distance_squared.sqrt();

                    let seperation_relative_weight = seperation_weight_single / distance;

                    acceleration_x += diff_x * seperation_relative_weight;
                    acceleration_y += diff_y * seperation_relative_weight;
                }

                accelerations_x[*i] = acceleration_x;
                accelerations_y[*i] = acceleration_y;
            }

        }

        for i in 0..self.boids.len() {
            self.boids[i].update(accelerations_x[i], accelerations_y[i]);
        }
    }
}

#[wasm_bindgen]
extern "C" {
    fn draw(c: u32, x: f32, y: f32);
    fn log(x: f32);
}
