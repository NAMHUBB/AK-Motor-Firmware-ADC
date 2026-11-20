#![no_std]

pub mod adc;
pub mod bridge;
pub mod config;
pub mod control;
pub mod cubemars;
pub mod dispatch;
pub mod extended;
pub mod monitor;
pub mod robotiq;

#[cfg(test)]
extern crate std;
#[cfg(test)]
mod adc_tests;
pub mod advanced;
#[cfg(test)]
mod advanced_tests;
#[cfg(test)]
mod ak60_tests;
#[cfg(test)]
mod dual_tests;
#[cfg(test)]
mod homing_tests;
#[cfg(test)]
mod hybrid_tests;
#[cfg(test)]
mod monitor_tests;
#[cfg(test)]
mod scaled_tests;
#[cfg(test)]
mod tests;
