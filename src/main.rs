use std::io::prelude::*;
use std::f32::consts::E;

#[derive(Debug, Clone, Copy)]
struct SpringConfig {
    pub mass: f32,
    pub stiffness: f32,
    pub damping: f32,
}

impl SpringConfig {
    fn critical_damping(mass: f32, stiffness: f32) -> Self {
        let ang_vel = (stiffness / mass).sqrt();
        let gamma = ang_vel;
        let damping = gamma * 2. * mass;

        Self {
            mass,
            stiffness,
            damping,
        }
    }
}

impl Default for SpringConfig {
    fn default() -> Self {
        Self {
            mass: 0.5,
            stiffness: 5.,
            damping: 0.5,
        }
    }
}

#[derive(Debug)]
struct Spring {
    gamma: f32,
    ang_vel_square: f32,
    pub velocity: f32,
    pub position: f32,
}

impl Spring {
    fn new(config: SpringConfig) -> Self {
        let SpringConfig {
            mass,
            stiffness,
            damping,
        } = config;

        let gamma = damping / (2. * mass);
        let ang_vel_square = stiffness / mass;

        Self {
            gamma,
            ang_vel_square,
            position: 0.,
            velocity: 0.,
        }
    }

    pub fn evaluate(&mut self, delta_t: f32) -> f32 {
        let gamma_sq = self.gamma * self.gamma;
        let (position, velocity) = if gamma_sq > self.ang_vel_square {
            self.overdampened(delta_t)
        } else if gamma_sq < self.ang_vel_square {
            self.underdampened(delta_t)
        } else {
            self.critical()
        };

        self.position = position * self.decay(delta_t);
        self.velocity = velocity * self.decay(delta_t);

        self.position
    }

    fn decay(&self, delta_t: f32) -> f32 {
        E.powf(-self.gamma * delta_t)
    }

    fn overdampened(&self, delta_t: f32) -> (f32, f32) {
        let Spring {
            gamma,
            ang_vel_square,
            position,
            velocity,
            ..
        } = self;

        let omega_d_i = (gamma * gamma - ang_vel_square).sqrt();
        let a = (position * (gamma - omega_d_i) + velocity) / (2. * omega_d_i);
        let b = position - a;

        let e_pow = E.powf(omega_d_i * delta_t);
        let b_over_e_pow = b / e_pow;

        let position = a * e_pow + b_over_e_pow;
        let velocity = omega_d_i * (a * e_pow - b_over_e_pow) - gamma * position;

        (position, velocity)
    }

    fn underdampened(&self, delta_t: f32) -> (f32, f32) {
        let Spring {
            gamma,
            ang_vel_square,
            position,
            velocity,
            ..
        } = self;

        let omega_d = (ang_vel_square - gamma * gamma).sqrt();

        let b = position;
        let a = (velocity + gamma * b) / omega_d;

        let phase = omega_d * delta_t;
        let position = a * f32::sin(phase) + b * f32::cos(-phase);
        let velocity =
            omega_d * (a * f32::cos(phase) - b * f32::sin(phase))
            - gamma * (a * f32::sin(phase) + b * f32::cos(phase));

        (position, velocity)
    }

    fn critical(&self) -> (f32, f32) {
        (self.position, -self.position / self.gamma)
    }
}

fn render(x: isize) {
    let mut line = [b' '; 81];

    let line = if let -40..=40 = x {
        let index = x + 40;
        line[index as usize] = b'.';

        line
    } else {
        line
    };

    let stdout = std::io::stdout();
    let mut stdout = stdout.lock();

    stdout.write_all(b"\r").unwrap();
    stdout.write_all(&line).unwrap();
    stdout.flush().unwrap();
}

fn position_to_isize(x: f32) -> isize {
    (x * 40.5) as isize
}

fn animate(mut spring: Spring) {
    let mut last_update = std::time::Instant::now();

    spring.evaluate(0.0);

    let mut last_index = position_to_isize(spring.position);

    loop {
        let now = std::time::Instant::now();
        let delta_t = (now - last_update).as_secs_f32();
        spring.evaluate(delta_t);

        let stopped = spring.position.abs() < 0.01 && spring.velocity.abs() < 0.01;

        if stopped {
            println!(
                "\nstopped at position = {}, velocity = {}",
                spring.position,
                spring.velocity,
            );
            return;
        }

        let new_index = position_to_isize(spring.position);

        if last_index != new_index {
            render(new_index);
        }

        last_update = now;
        last_index = new_index;

        std::thread::sleep(std::time::Duration::from_millis(15));
    }
}

fn main() {
    let mut critical = SpringConfig::critical_damping(1., 10.);
    critical.damping = 1.;

    let mut spring = Spring::new(critical);

    spring.position = 1.;
    spring.velocity = 0.;

    animate(spring);
}
