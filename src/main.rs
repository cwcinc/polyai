#![allow(unsafe_op_in_unsafe_fn)]
#![allow(improper_ctypes)]
use autocxx::prelude::*;

include_cpp! {
    #include "ammo.h"
    safety!(unsafe)
    generate!("btVector3")
}

fn main() {
    use std::time::Instant;
    let vec = ffi::btVector3::new1(&1.0, &2.0, &3.0).within_unique_ptr();
    let now = Instant::now();
    let mut cum = 0.0;
    for _ in 0..10000 {
        let length = vec.length();
        cum += length;
    }
    let duration = now.elapsed();
    println!("Length: {}, Time taken: {:?}", cum, duration);
}
