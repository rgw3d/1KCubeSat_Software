#![no_std]
#![feature(custom_test_frameworks)]
#![test_runner(crate::test_runner)]
#![feature(default_free_fn)]

#[cfg(feature = "std")]
extern crate std;
#[cfg(feature = "std")]
use std::println;
#[cfg(feature = "std")]
#[macro_use]
extern crate assert_float_eq;

extern crate transpose;

// Simply a wrapper for the transpose library
pub fn matrix_transpose<T: Copy>(
    input: &[T],
    output: &mut [T],
    input_height: usize,
    input_width: usize,
) {
    // Note the ordering of height & width.
    transpose::transpose(input, output, input_width, input_height);
}

// width ==# of columns
// height ==# of rows
// a_cols_w must equal b_rows_h
pub fn multiply<T: Copy + core::ops::Mul<Output = T> + core::ops::Add<Output = T>>(
    a: &[T],
    b: &[T],
    out: &mut [T],
    a_rows_h: usize,
    a_cols_w: usize,
    b_rows_h: usize,
    b_cols_w: usize,
) {
    // Break early if dimensions are bad.
    // Or break if any dimensions are equal to 0
    if (a_cols_w != b_rows_h)
        || ((a_rows_h == 0) || (a_cols_w == 0) || (b_rows_h == 0) || (b_cols_w == 0))
    {
        return;
    }

    // Do matrix multiplication
    for i in 0..a_rows_h {
        for j in 0..b_cols_w {
            unsafe {
                let mut sum: T = *a.get_unchecked(i * a_cols_w) * *b.get_unchecked(j);
                for k in 1..b_rows_h {
                    sum = sum
                        + *a.get_unchecked(i * a_cols_w + k) * *b.get_unchecked(k * b_cols_w + j);
                }
                *out.get_unchecked_mut(i * b_cols_w + j) = sum;
            }
        }
    }
}

pub fn inverse_3x3<
    T: Copy
        + core::ops::Mul<Output = T>
        + core::ops::Add<Output = T>
        + core::ops::Sub<Output = T>
        + core::ops::Div<Output = T>
        + core::cmp::PartialEq
        + core::default::Default,
>(
    input: &[T; 9],
    out: &mut [T; 9],
) -> core::result::Result<(), ()> {
    let det: T = det_3x3(input);

    // bail if the determinant is zero.
    // Can't find the inverse if this is true.
    if det == core::default::default() {
        return Err(());
    }

    out[0] = (input[4] * input[8]) - (input[5] * input[7]);
    out[1] = (input[2] * input[7]) - (input[1] * input[8]);
    out[2] = (input[1] * input[5]) - (input[2] * input[4]);

    out[3] = (input[5] * input[6]) - (input[3] * input[8]);
    out[4] = (input[0] * input[8]) - (input[2] * input[6]);
    out[5] = (input[2] * input[3]) - (input[0] * input[5]);

    out[6] = (input[3] * input[7]) - (input[4] * input[6]);
    out[7] = (input[1] * input[6]) - (input[0] * input[7]);
    out[8] = (input[0] * input[4]) - (input[1] * input[3]);

    matrix_divide_scalar(out, det, 3, 3);
    Ok(())
}

pub fn det_3x3<
    T: Copy + core::ops::Mul<Output = T> + core::ops::Add<Output = T> + core::ops::Sub<Output = T>,
>(
    input: &[T; 9],
) -> T {
    // a(ei−fh)−b(di−fg)+c(dh−eg)
    // aei+bfg+cdh−afh−bdi−ceg
    (input[0] * input[4] * input[8])
        + (input[1] * input[5] * input[6])
        + (input[2] * input[3] * input[7])
        - (input[0] * input[5] * input[7])
        - (input[1] * input[3] * input[8])
        - (input[2] * input[4] * input[6])
}

pub fn matrix_multiply_scalar<T: Copy + core::ops::Mul<Output = T>>(
    out: &mut [T],
    scalar: T,
    rows_h: usize,
    cols_w: usize,
) {
    for i in 0..(rows_h * cols_w) {
        unsafe {
            *out.get_unchecked_mut(i) = *out.get_unchecked(i) * scalar;
        }
    }
}

pub fn matrix_divide_scalar<T: Copy + core::ops::Div<Output = T>>(
    out: &mut [T],
    scalar: T,
    rows_h: usize,
    cols_w: usize,
) {
    for i in 0..(rows_h * cols_w) {
        unsafe {
            *out.get_unchecked_mut(i) = *out.get_unchecked(i) / scalar;
        }
    }
}

//////
//////
//////

#[cfg(feature = "std")]
macro_rules! function_name {
    () => {{
        fn f() {}
        fn type_name_of<T>(_: T) -> &'static str {
            std::any::type_name::<T>()
        }
        let name = type_name_of(f);

        // Find and cut the rest of the path
        match &name[..name.len() - 3].rfind(':') {
            Some(pos) => &name[pos + 1..name.len() - 3],
            None => &name[..name.len() - 3],
        }
    }};
}

#[cfg(test)]
fn test_runner(tests: &[&dyn Fn()]) {
    println!("Running {} tests", tests.len());
    for test in tests {
        test();
    }
}

#[test_case]
fn test_multiply_1x1_1x1() {
    println!("Start ==== {}", function_name!());
    let a: [u16; 1] = [1; 1];
    let b: [u16; 1] = [2; 1];
    let mut out: [u16; 1] = [0; 1];
    multiply(&a, &b, &mut out, 1, 1, 1, 1);
    assert_eq!(out[0], 2);
}

#[test_case]
fn test_multiply_2x2_2x2() {
    println!("Start ==== {}", function_name!());
    // Test super basic 2x2
    let a: [u16; 4] = [1, 1, 1, 1];
    let b: [u16; 4] = [4, 4, 4, 4];
    let mut out: [u16; 4] = [0; 4];
    multiply(&a, &b, &mut out, 2, 2, 2, 2);
    assert_eq!(out[0], 8);
    assert_eq!(out[1], 8);
    assert_eq!(out[2], 8);
    assert_eq!(out[3], 8);

    // Test some more bigger non 1 values
    let a: [u32; 4] = [10, 111, 2000, 89];
    let b: [u32; 4] = [882, 100, 123456, 1];
    let mut out: [u32; 4] = [0; 4];
    multiply(&a, &b, &mut out, 2, 2, 2, 2);
    assert_eq!(out[0], 13712436);
    assert_eq!(out[1], 1111);
    assert_eq!(out[2], 12751584);
    assert_eq!(out[3], 200089);

    // Test some negative values
    let a: [i32; 4] = [-13, 200, -3500, 77];
    let b: [i32; 4] = [100, 34, 67, -23];
    let mut out: [i32; 4] = [0; 4];
    multiply(&a, &b, &mut out, 2, 2, 2, 2);
    assert_eq!(out[0], 12100);
    assert_eq!(out[1], -5042);
    assert_eq!(out[2], -344841);
    assert_eq!(out[3], -120771);
}

#[test_case]
fn test_multiply_invalid_size() {
    println!("Start ==== {}", function_name!());
    let a: [u16; 4] = [1, 1, 1, 1];
    let b: [u16; 4] = [4, 4, 4, 4];
    let mut out: [u16; 4] = [0; 4];
    // Bad array size
    multiply(&a, &b, &mut out, 2, 3, 2, 2);
    // out array should not have changed.
    assert_eq!(out[0], 0);
    assert_eq!(out[1], 0);
    assert_eq!(out[2], 0);
    assert_eq!(out[3], 0);

    // size of 0
    multiply(&a, &b, &mut out, 0, 2, 2, 2);
    // out array should not have changed.
    assert_eq!(out[0], 0);
    assert_eq!(out[1], 0);
    assert_eq!(out[2], 0);
    assert_eq!(out[3], 0);

    // size of 0
    multiply(&a, &b, &mut out, 2, 0, 0, 2);
    // out array should not have changed.
    assert_eq!(out[0], 0);
    assert_eq!(out[1], 0);
    assert_eq!(out[2], 0);
    assert_eq!(out[3], 0);

    // size of 0
    multiply(&a, &b, &mut out, 2, 2, 2, 0);
    // out array should not have changed.
    assert_eq!(out[0], 0);
    assert_eq!(out[1], 0);
    assert_eq!(out[2], 0);
    assert_eq!(out[3], 0);
}

#[test_case]
fn test_multiply_10x1_1x10() {
    println!("Start ==== {}", function_name!());
    let a: [u16; 10] = [5; 10];
    let b: [u16; 10] = [10; 10];
    let mut out: [u16; 100] = [0; 100];
    multiply(&a, &b, &mut out, 10, 1, 1, 10);
    for i in 0..100 {
        assert_eq!(out[i], 50);
    }
}

#[test_case]
fn test_multiply_1x10_10x1() {
    println!("Start ==== {}", function_name!());
    let a: [u16; 10] = [5; 10];
    let b: [u16; 10] = [10; 10];
    let mut out: [u16; 1] = [0; 1];
    multiply(&a, &b, &mut out, 1, 10, 10, 1);
    assert_eq!(out[0], 500);
}

#[test_case]
fn test_multiply_3x5_5x8() {
    println!("Start ==== {}", function_name!());
    let a: [i32; 15] = [3; 15];
    let b: [i32; 40] = [5; 40];
    let mut out: [i32; 24] = [0; 24];
    multiply(&a, &b, &mut out, 3, 5, 5, 8);
    for i in 0..24 {
        assert_eq!(out[i], 75);
    }
}

#[test_case]
fn test_transpose_16x3() {
    println!("Start ==== {}", function_name!());
    let mut a: [usize; 48] = [0; 48];
    for i in 0..48 {
        a[i] = i + 1; // fill array with increasing values
    }
    let mut out: [usize; 48] = [0; 48];
    matrix_transpose(&a, &mut out, 16, 3);
    for i in 0..16 {
        assert_eq!(out[i], i * 3 + 1);
    }
    for i in 16..32 {
        assert_eq!(out[i], (i - 16) * 3 + 2);
    }
    for i in 32..48 {
        assert_eq!(out[i], (i - 32) * 3 + 3);
    }
}

#[test_case]
fn test_det_3x3() {
    println!("Start ==== {}", function_name!());
    let mut a: [i32; 9] = [0; 9];
    for i in 0..9 {
        a[i] = i as i32 * 10;
    }
    assert_eq!(det_3x3(&mut a), 0);

    // Again but with different values
    a = [1, -20, 34, 8, 99, 1000, 77, 59, 0];
    assert_eq!(det_3x3(&mut a), -1842134);
}

#[test_case]
fn test_multiply_scalar() {
    println!("Start ==== {}", function_name!());
    let mut a: [i32; 9] = [0; 9];
    for i in 0..9 {
        a[i] = i as i32;
    }
    let scalar: i32 = 11;
    matrix_multiply_scalar(&mut a, scalar, 3, 3);
    for i in 0..9 {
        assert_eq!(a[i], i as i32 * scalar)
    }
}

#[test_case]
fn test_multiply_divide() {
    println!("Start ==== {}", function_name!());
    let mut a: [i32; 9] = [0; 9];
    for i in 0..9 {
        a[i] = i as i32 * 10;
    }
    let scalar: i32 = 10;
    matrix_divide_scalar(&mut a, scalar, 3, 3);
    for i in 0..9 {
        assert_eq!(a[i], ((i as i32 * 10) / scalar));
    }
}

#[test_case]
fn test_multiply_inverse_3x3() {
    println!("Start ==== {}", function_name!());
    // Test an all 1 matrix first, which doesn't have an inverse because the det is 0
    let a: [i32; 9] = [1; 9];
    let mut out: [i32; 9] = [0; 9];
    assert_eq!(inverse_3x3(&a, &mut out), Err(()));

    // Test with a matrix that has a determinant
    let a: [i32; 9] = [1, 1, 1, 10, 20, 30, 2, -3, 1];
    let mut out: [i32; 9] = [0; 9];
    assert_eq!(inverse_3x3(&a, &mut out), Ok(()));
    // because integer, here's what we get
    assert_eq!(out[0], 1);
    assert_eq!(out[1], 0);
    assert_eq!(out[2], 0);
    assert_eq!(out[3], 0);
    assert_eq!(out[4], 0);
    assert_eq!(out[5], 0);
    assert_eq!(out[6], 0);
    assert_eq!(out[7], 0);
    assert_eq!(out[8], 0);

    // Using floats
    // Test with a matrix that has a determinant
    let a: [f32; 9] = [1.0, 1.0, 1.0, 10.0, 20.0, 30.0, 2.0, -3.0, 1.0];
    let mut out: [f32; 9] = [0.0; 9];
    assert_eq!(inverse_3x3(&a, &mut out), Ok(()));
    assert_f32_near!(out[0], 1.222222222222);
    assert_f32_near!(out[1], -0.044444444444);
    assert_f32_near!(out[2], 0.111111111);
    assert_f32_near!(out[3], 0.555555555);
    assert_f32_near!(out[4], -0.0111111111);
    assert_f32_near!(out[5], -0.2222222222);
    assert_f32_near!(out[6], -0.7777777777);
    assert_f32_near!(out[7], 0.055555555555);
    assert_f32_near!(out[8], 0.111111111111);
}

#[test_case]
fn test_left_pseudo_inverse() {
    println!("Start ==== {}", function_name!());
    let mut matrix_a_out: [f32; 48] = [0.0; 48];
    {
        let mut matrix_a: [f32; 48] = [0.0; 48];
        let adc_coord_map: [(i32, i32); 16] = [
            (2, 2),
            (2, 1),
            (1, 2),
            (1, 1),
            //
            (-1, 2),
            (-1, 1),
            (-2, 2),
            (-2, 1),
            //
            (-1, -1),
            (-1, -2),
            (-2, -1),
            (-2, -2),
            //
            (2, -1),
            (2, 2),
            (1, -1),
            (1, -2),
        ];
        // populate matrix a
        for i in 0..16 {
            let (a, b) = adc_coord_map[i];
            matrix_a[(i * 3) + 0] = a as f32;
            matrix_a[(i * 3) + 1] = b as f32;
            matrix_a[(i * 3) + 2] = 1.0;
        }

        let mut matrix_a_t: [f32; 48] = [0.0; 48];
        matrix_transpose(&matrix_a, &mut matrix_a_t, 16, 3);
        let mut matrix_tmp: [f32; 9] = [0.0; 9];
        multiply(&matrix_a_t, &matrix_a, &mut matrix_tmp, 3, 16, 16, 3);
        let mut matrix_a_t_mul_inv: [f32; 9] = [0.0; 9];
        match inverse_3x3(&matrix_tmp, &mut matrix_a_t_mul_inv) {
            Ok(()) => println!("inverse OK"),
            Err(()) => panic!("inverse not ok"),
        }
        multiply(
            &matrix_a_t_mul_inv,
            &matrix_a_t,
            &mut matrix_a_out,
            3,
            3,
            3,
            16,
        );
    }
}
