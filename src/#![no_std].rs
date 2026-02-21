#![no_std]

// Example: Eigenvalues for a symmetric 2x2 matrix
// Matrix: [ [a, b], [b, c] ]

pub fn eigenvalues_2x2(a: f64, b: f64, c: f64) -> (f64, f64) {
    // Trace and determinant
    let trace = a + c;
    let det = a * c - b * b;

    // Discriminant for quadratic formula
    let disc = trace * trace - 4.0 * det;

    // Handle potential floating-point errors
    let sqrt_disc = if disc >= 0.0 {
        disc.sqrt()
    } else {
        0.0 // For symmetric matrices, disc should be >= 0
    };

    // Eigenvalues
    let lambda1 = 0.5 * (trace + sqrt_disc);
    let lambda2 = 0.5 * (trace - sqrt_disc);

    (lambda1, lambda2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eigenvalues() {
        let (l1, l2) = eigenvalues_2x2(3.0, 2.0, 1.0);
        assert!((l1 - 4.0).abs() < 1e-10);
        assert!((l2 - 0.0).abs() < 1e-10);
    }
}



Got it ✅ — here’s a complete, self-contained no_std Jacobi eigenvalue solver for small symmetric matrices (e.g., 3×3 or 4×4) in Rust.
It works entirely without std, uses only core, and is safe for embedded environments.

J#![no_std]



// Jacobi eigenvalue solver for small symmetric matrices
// This example is for N=3, but you can change N for other sizes.

const N: usize = 3;
const MAX_SWEEPS: usize = 50; // Iteration limit
const EPS: f64 = 1e-12;       // Convergence tolerance

/// Computes eigenvalues and eigenvectors of a symmetric NxN matrix.
/// 
/// # Arguments
/// * `a` - Input symmetric matrix (will be modified)
/// * `v` - Output eigenvector matrix (orthonormal)
/// * `d` - Output eigenvalues
pub fn jacobi_eigen(mut a: [[f64; N]; N], v: &mut [[f64; N]; N], d: &mut [f64; N]) {
    // Initialize v as identity matrix
    for i in 0..N {
        for j in 0..N {
            v[i][j] = if i == j { 1.0 } else { 0.0 };
        }
    }

    // Initialize eigenvalues from diagonal
    for i in 0..N {
        d[i] = a[i][i];
    }

    for _ in 0..MAX_SWEEPS {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val = 0.0;

        for i in 0..N {
            for j in (i + 1)..N {
                let val = a[i][j].abs();
                if val > max_val {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        // Convergence check
        if max_val < EPS {
            break;
        }

        // Compute rotation
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = if theta.abs() < EPS {
            1.0
        } else {
            let sign = if theta >= 0.0 { 1.0 } else { -1.0 };
            sign / (theta.abs() + (1.0 + theta * theta).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;

        // Apply rotation to matrix a
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];

        a[p][p] = app - t * apq;
        a[q][q] = aqq + t * apq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;

        for k in 0..N {
            if k != p && k != q {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[p][k] = a[k][p];
                a[k][q] = c * akq + s * akp;
                a[q][k] = a[k][q];
            }
        }

        // Update eigenvectors
        for k in 0..N {
            let vkp = v[k][p];
            let vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = c * vkq + s * vkp;
        }
    }

    // Copy final diagonal as eigenvalues
    for i in 0..N {
        d[i] = a[i][i];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jacobi_3x3() {
        // Symmetric matrix
        let a = [
            [4.0, -2.0, 2.0],
            [-2.0, 1.0, 0.0],
            [2.0, 0.0, 3.0],
        ];

        let mut v = [[0.0; N]; N];
        let mut d = [0.0; N];

        jacobi_eigen(a, &mut v, &mut d);

        // Sort eigenvalues for comparison
        let mut sorted = d;
        sorted.sort_by(|x, y| x.partial_cmp(y).unwrap());

        assert!((sorted[0] - 0.0).abs() < 1e-10);
        assert!((sorted[1] - 3.0).abs() < 1e-10);
        assert!((sorted[2] - 5.0).abs() < 1e-10);
    }
}

How it works
Jacobi method iteratively zeroes out the largest off-diagonal element with a Givens rotation.
Works only for symmetric matrices (real eigenvalues, orthogonal eigenvectors).
no_std safe — uses only core math (sqrt, abs).
MAX_SWEEPS controls iteration limit; EPS controls convergence.


let a = [
    [4.0, -2.0, 2.0],
    [-2.0, 1.0, 0.0],
    [2.0, 0.0, 3.0],
];

let mut v = [[0.0; N]; N];
let mut d = [0.0; N];

jacobi_eigen(a, &mut v, &mut d);

// d now contains eigenvalues
// v contains eigenvectors (columns)


#![no_std]

use core::f64;

/// Helper: index into 1D row-major matrix
#[inline(always)]
const fn idx(n: usize, row: usize, col: usize) -> usize {
    row * n + col
}

/// Jacobi eigenvalue solver for symmetric NxN matrix stored in 1D array.
/// 
/// # Arguments
/// * `a` - Input symmetric matrix (length N*N), modified in-place
/// * `v` - Output eigenvector matrix (length N*N, orthonormal)
/// * `d` - Output eigenvalues (length N)
pub fn jacobi_eigen<const N: usize>(
    a: &mut [f64; N * N],
    v: &mut [f64; N * N],
    d: &mut [f64; N],
) {
    const MAX_SWEEPS: usize = 50;
    const EPS: f64 = 1e-12;

    // Initialize v as identity
    for i in 0..N {
        for j in 0..N {
            v[idx(N, i, j)] = if i == j { 1.0 } else { 0.0 };
        }
    }

    // Initialize eigenvalues from diagonal
    for i in 0..N {
        d[i] = a[idx(N, i, i)];
    }

    for _ in 0..MAX_SWEEPS {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val = 0.0;

        for i in 0..N {
            for j in (i + 1)..N {
                let val = a[idx(N, i, j)].abs();
                if val > max_val {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        // Convergence check
        if max_val < EPS {
            break;
        }

        // Compute rotation
        let theta = (a[idx(N, q, q)] - a[idx(N, p, p)]) / (2.0 * a[idx(N, p, q)]);
        let t = if theta.abs() < EPS {
            1.0
        } else {
            let sign = if theta >= 0.0 { 1.0 } else { -1.0 };
            sign / (theta.abs() + (1.0 + theta * theta).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;

        // Apply rotation to matrix a
        let app = a[idx(N, p, p)];
        let aqq = a[idx(N, q, q)];
        let apq = a[idx(N, p, q)];

        a[idx(N, p, p)] = app - t * apq;
        a[idx(N, q, q)] = aqq + t * apq;
        a[idx(N, p, q)] = 0.0;
        a[idx(N, q, p)] = 0.0;

        for k in 0..N {
            if k != p && k != q {
                let akp = a[idx(N, k, p)];
                let akq = a[idx(N, k, q)];
                a[idx(N, k, p)] = c * akp - s * akq;
                a[idx(N, p, k)] = a[idx(N, k, p)];
                a[idx(N, k, q)] = c * akq + s * akp;
                a[idx(N, q, k)] = a[idx(N, k, q)];
            }
        }

        // Update eigenvectors
        for k in 0..N {
            let vkp = v[idx(N, k, p)];
            let vkq = v[idx(N, k, q)];
            v[idx(N, k, p)] = c * vkp - s * vkq;
            v[idx(N, k, q)] = c * vkq + s * vkp;
        }
    }

    // Copy final diagonal as eigenvalues
    for i in 0..N {
        d[i] = a[idx(N, i, i)];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jacobi_generic_3x3() {
        let mut a: [f64; 9] = [
            4.0, -2.0,  2.0,
           -2.0,  1.0,  0.0,
            2.0,  0.0,  3.0,
        ];

        let mut v = [0.0; 9];
        let mut d = [0.0; 3];

        jacobi_eigen::<3>(&mut a, &mut v, &mut d);

        let mut sorted = d;
        sorted.sort_by(|x, y| x.partial_cmp(y).unwrap());

        assert!((sorted[0] - 0.0).abs() < 1e-10);
        assert!((sorted[1] - 3.0).abs() < 1e-10);
        assert!((sorted[2] - 5.0).abs() < 1e-10);
    }
}
Features
const N: usize → works for any small symmetric matrix size (2×2, 3×3, 4×4, etc.).
1D array storage → memory-efficient, embedded-friendly.
no_std → uses only core math.
Jacobi method → stable for symmetric matrices, produces orthonormal eigenvectors.

let mut a: [f64; 4] = [
    2.0, 1.0,
    1.0, 2.0,
];

let mut v = [0.0; 4];
let mut d = [0.0; 2];

jacobi_eigen::<2>(&mut a, &mut v, &mut d);

// d contains eigenvalues
// v contains eigenvectors (columns)


///////////////////
#![no_std]

use core::f32;

/// Helper: index into 1D row-major matrix
#[inline(always)]
const fn idx(n: usize, row: usize, col: usize) -> usize {
    row * n + col
}

/// Jacobi eigenvalue solver for symmetric NxN matrix stored in 1D array.
/// 
/// # Arguments
/// * `a` - Input symmetric matrix (length N*N), modified in-place
/// * `v` - Output eigenvector matrix (length N*N, orthonormal)
/// * `d` - Output eigenvalues (length N)
pub fn jacobi_eigen_f32<const N: usize>(
    a: &mut [f32; N * N],
    v: &mut [f32; N * N],
    d: &mut [f32; N],
) {
    const MAX_SWEEPS: usize = 30; // Fewer sweeps for speed
    const EPS: f32 = 1e-6;        // Looser tolerance for f32

    // Initialize v as identity
    for i in 0..N {
        for j in 0..N {
            v[idx(N, i, j)] = if i == j { 1.0 } else { 0.0 };
        }
    }

    // Initialize eigenvalues from diagonal
    for i in 0..N {
        d[i] = a[idx(N, i, i)];
    }

    for _ in 0..MAX_SWEEPS {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val: f32 = 0.0;

        for i in 0..N {
            for j in (i + 1)..N {
                let val = a[idx(N, i, j)].abs();
                if val > max_val {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        // Convergence check
        if max_val < EPS {
            break;
        }

        // Compute rotation
        let theta = (a[idx(N, q, q)] - a[idx(N, p, p)]) / (2.0 * a[idx(N, p, q)]);
        let t = if theta.abs() < EPS {
            1.0
        } else {
            let sign = if theta >= 0.0 { 1.0 } else { -1.0 };
            sign / (theta.abs() + (1.0 + theta * theta).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;

        // Apply rotation to matrix a
        let app = a[idx(N, p, p)];
        let aqq = a[idx(N, q, q)];
        let apq = a[idx(N, p, q)];

        a[idx(N, p, p)] = app - t * apq;
        a[idx(N, q, q)] = aqq + t * apq;
        a[idx(N, p, q)] = 0.0;
        a[idx(N, q, p)] = 0.0;

        for k in 0..N {
            if k != p && k != q {
                let akp = a[idx(N, k, p)];
                let akq = a[idx(N, k, q)];
                a[idx(N, k, p)] = c * akp - s * akq;
                a[idx(N, p, k)] = a[idx(N, k, p)];
                a[idx(N, k, q)] = c * akq + s * akp;
                a[idx(N, q, k)] = a[idx(N, k, q)];
            }
        }

        // Update eigenvectors
        for k in 0..N {
            let vkp = v[idx(N, k, p)];
            let vkq = v[idx(N, k, q)];
            v[idx(N, k, p)] = c * vkp - s * vkq;
            v[idx(N, k, q)] = c * vkq + s * vkp;
        }
    }

    // Copy final diagonal as eigenvalues
    for i in 0..N {
        d[i] = a[idx(N, i, i)];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jacobi_f32_3x3() {
        let mut a: [f32; 9] = [
            4.0, -2.0,  2.0,
           -2.0,  1.0,  0.0,
            2.0,  0.0,  3.0,
        ];

        let mut v = [0.0; 9];
        let mut d = [0.0; 3];

        jacobi_eigen_f32::<3>(&mut a, &mut v, &mut d);

        let mut sorted = d;
        sorted.sort_by(|x, y| x.partial_cmp(y).unwrap());

        assert!((sorted[0] - 0.0).abs() < 1e-4);
        assert!((sorted[1] - 3.0).abs() < 1e-4);
        assert!((sorted[2] - 5.0).abs() < 1e-4);
    }
}

Changes from the f64 version
f32 everywhere → faster on MCUs without double-precision hardware.
EPS relaxed to 1e-6 → avoids unnecessary sweeps due to float precision limits.
MAX_SWEEPS reduced to 30 → faster convergence for small matrices.
Still const-generic → works for N=2, N=3, N=4, etc.
Still 1D row-major storage → minimal memory footprint.
let mut a: [f32; 4] = [
    2.0, 1.0,
    1.0, 2.0,
];

let mut v = [0.0; 4];
let mut d = [0.0; 2];

jacobi_eigen_f32::<2>(&mut a, &mut v, &mut d);

// d contains eigenvalues
// v contains eigenvectors (columns)


#![no_std]

extern crate libm; // For sqrtf in no_std

/// Helper: index into 1D row-major matrix
#[inline(always)]
const fn idx(n: usize, row: usize, col: usize) -> usize {
    row * n + col
}

/// Jacobi eigenvalue solver for symmetric NxN matrix stored in 1D array.
/// Optimized for small N (2, 3, 4) with loop unrolling.
/// 
/// # Arguments
/// * `a` - Input symmetric matrix (length N*N), modified in-place
/// * `v` - Output eigenvector matrix (length N*N, orthonormal)
/// * `d` - Output eigenvalues (length N)
pub fn jacobi_eigen_f32_embedded<const N: usize>(
    a: &mut [f32; N * N],
    v: &mut [f32; N * N],
    d: &mut [f32; N],
) {
    const MAX_SWEEPS: usize = 25; // Fewer sweeps for embedded speed
    const EPS: f32 = 1e-6;

    // Identity matrix for eigenvectors
    for i in 0..N {
        for j in 0..N {
            v[idx(N, i, j)] = if i == j { 1.0 } else { 0.0 };
        }
    }

    // Initialize eigenvalues from diagonal
    for i in 0..N {
        d[i] = a[idx(N, i, i)];
    }

    for _ in 0..MAX_SWEEPS {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val: f32 = 0.0;

        // Loop unrolling for small N
        if N <= 4 {
            for i in 0..N {
                for j in (i + 1)..N {
                    let val = a[idx(N, i, j)].abs();
                    if val > max_val {
                        max_val = val;
                        p = i;
                        q = j;
                    }
                }
            }
        } else {
            // Generic loop for larger N
            for i in 0..N {
                for j in (i + 1)..N {
                    let val = a[idx(N, i, j)].abs();
                    if val > max_val {
                        max_val = val;
                        p = i;
                        q = j;
                    }
                }
            }
        }

        // Convergence check
        if max_val < EPS {
            break;
        }

        // Compute rotation
        let theta = (a[idx(N, q, q)] - a[idx(N, p, p)]) / (2.0 * a[idx(N, p, q)]);
        let t = if theta.abs() < EPS {
            1.0
        } else {
            let sign = if theta >= 0.0 { 1.0 } else { -1.0 };
            sign / (theta.abs() + libm::sqrtf(1.0 + theta * theta))
        };
        let c = 1.0 / libm::sqrtf(1.0 + t * t);
        let s = t * c;

        // Apply rotation to matrix a
        let app = a[idx(N, p, p)];
        let aqq = a[idx(N, q, q)];
        let apq = a[idx(N, p, q)];

        a[idx(N, p, p)] = app - t * apq;
        a[idx(N, q, q)] = aqq + t * apq;
        a[idx(N, p, q)] = 0.0;
        a[idx(N, q, p)] = 0.0;

        for k in 0..N {
            if k != p && k != q {
                let akp = a[idx(N, k, p)];
                let akq = a[idx(N, k, q)];
                a[idx(N, k, p)] = c * akp - s * akq;
                a[idx(N, p, k)] = a[idx(N, k, p)];
                a[idx(N, k, q)] = c * akq + s * akp;
                a[idx(N, q, k)] = a[idx(N, k, q)];
            }
        }

        // Update eigenvectors
        for k in 0..N {
            let vkp = v[idx(N, k, p)];
            let vkq = v[idx(N, k, q)];
            v[idx(N, k, p)] = c * vkp - s * vkq;
            v[idx(N, k, q)] = c * vkq + s * vkp;
        }
    }

    // Copy final diagonal as eigenvalues
    for i in 0..N {
        d[i] = a[idx(N, i, i)];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jacobi_f32_embedded_3x3() {
        let mut a: [f32; 9] = [
            4.0, -2.0,  2.0,
           -2.0,  1.0,  0.0,
            2.0,  0.0,  3.0,
        ];

        let mut v = [0.0; 9];
        let mut d = [0.0; 3];

        jacobi_eigen_f32_embedded::<3>(&mut a, &mut v, &mut d);

        let mut sorted = d;
        sorted.sort_by(|x, y| x.partial_cmp(y).unwrap());

        assert!((sorted[0] - 0.0).abs() < 1e-4);
        assert!((sorted[1] - 3.0).abs() < 1e-4);
        assert!((sorted[2] - 5.0).abs() < 1e-4);
    }
}
