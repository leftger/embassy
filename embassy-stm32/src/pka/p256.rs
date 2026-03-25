//! P-256 (secp256r1) ECDH key agreement using the PKA hardware accelerator.
//!
//! Provides a high-level API for Elliptic Curve Diffie-Hellman (ECDH) on the
//! NIST P-256 curve, fully hardware-accelerated via the on-chip PKA peripheral.
//!
//! # Example
//!
//! ```no_run
//! use embassy_stm32::pka::Pka;
//! use embassy_stm32::pka::p256::{PublicKey, SharedSecret};
//!
//! let mut pka = Pka::new_blocking(p.PKA, Irqs);
//! let mut rng = Rng::new(p.RNG, Irqs);
//!
//! // Generate a random private key
//! let mut private_key = [0u8; 32];
//! rng.fill_bytes(&mut private_key);
//!
//! // Derive the public key using PKA hardware
//! let public_key = pka.p256_public_key(&private_key)?;
//!
//! // Parse peer's public key from SEC1 uncompressed encoding
//! let peer_public = PublicKey::from_sec1_uncompressed(&peer_bytes)?;
//!
//! // Validate peer's key is on the curve
//! assert!(pka.p256_validate_key(&peer_public)?);
//!
//! // Compute shared secret using PKA hardware
//! let shared = pka.p256_shared_secret(&private_key, &peer_public)?;
//! let secret_bytes: &[u8; 32] = shared.as_bytes();
//! ```
//!
//! # Security Notes
//!
//! - Private keys must be generated using a cryptographically secure RNG.
//! - Always validate received public keys with [`Pka::p256_validate_key`]
//!   before computing shared secrets.
//! - Use a KDF (e.g., HKDF-SHA256) to derive session keys from the raw shared
//!   secret — do **not** use the raw bytes directly as an encryption key.

use super::{EccPoint, EcdsaCurveParams, Error, Instance, Pka};

/// Size of a P-256 scalar or coordinate in bytes.
pub const KEY_SIZE: usize = 32;

/// Size of a SEC1 uncompressed public key (0x04 ‖ x ‖ y).
pub const SEC1_UNCOMPRESSED_SIZE: usize = 1 + KEY_SIZE + KEY_SIZE; // 65

/// P-256 public key (affine point).
pub struct PublicKey {
    x: [u8; KEY_SIZE],
    y: [u8; KEY_SIZE],
}

impl PublicKey {
    /// Create from raw x/y coordinates (big-endian, 32 bytes each).
    pub fn from_xy(x: &[u8; KEY_SIZE], y: &[u8; KEY_SIZE]) -> Self {
        Self { x: *x, y: *y }
    }

    /// Parse from SEC1 uncompressed encoding (`0x04 ‖ x ‖ y`, 65 bytes).
    ///
    /// Returns [`Error::InvalidSize`] if the tag byte is not `0x04` or the
    /// slice length is wrong.
    pub fn from_sec1_uncompressed(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() != SEC1_UNCOMPRESSED_SIZE || bytes[0] != 0x04 {
            return Err(Error::InvalidSize);
        }
        let mut x = [0u8; KEY_SIZE];
        let mut y = [0u8; KEY_SIZE];
        x.copy_from_slice(&bytes[1..33]);
        y.copy_from_slice(&bytes[33..65]);
        Ok(Self { x, y })
    }

    /// Serialize to SEC1 uncompressed encoding (`0x04 ‖ x ‖ y`, 65 bytes).
    pub fn to_sec1_uncompressed(&self) -> [u8; SEC1_UNCOMPRESSED_SIZE] {
        let mut out = [0u8; SEC1_UNCOMPRESSED_SIZE];
        out[0] = 0x04;
        out[1..33].copy_from_slice(&self.x);
        out[33..65].copy_from_slice(&self.y);
        out
    }

    /// X-coordinate (big-endian).
    pub fn x(&self) -> &[u8; KEY_SIZE] {
        &self.x
    }

    /// Y-coordinate (big-endian).
    pub fn y(&self) -> &[u8; KEY_SIZE] {
        &self.y
    }
}

/// ECDH shared secret (x-coordinate of the shared point).
pub struct SharedSecret {
    raw: [u8; KEY_SIZE],
}

impl SharedSecret {
    /// Raw shared-secret bytes (big-endian x-coordinate).
    pub fn as_bytes(&self) -> &[u8; KEY_SIZE] {
        &self.raw
    }
}

// ---------------------------------------------------------------------------
// Methods on Pka
// ---------------------------------------------------------------------------

impl<'d, T: Instance> Pka<'d, T> {
    /// Derive the P-256 public key for a given private key.
    ///
    /// Computes `public = private_key × G` where *G* is the P-256 generator,
    /// using the PKA hardware scalar multiplier.
    ///
    /// `private_key` must be a 32-byte big-endian scalar in the range `[1, n-1]`.
    pub fn p256_public_key(&mut self, private_key: &[u8; KEY_SIZE]) -> Result<PublicKey, Error> {
        let curve = EcdsaCurveParams::nist_p256();
        let mut point = EccPoint::new(KEY_SIZE);

        self.ecc_mul(
            &curve,
            private_key,
            curve.generator_x,
            curve.generator_y,
            &mut point,
        )?;

        let mut pk = PublicKey {
            x: [0u8; KEY_SIZE],
            y: [0u8; KEY_SIZE],
        };
        pk.x.copy_from_slice(&point.x[..KEY_SIZE]);
        pk.y.copy_from_slice(&point.y[..KEY_SIZE]);
        Ok(pk)
    }

    /// Compute the P-256 ECDH shared secret.
    ///
    /// Computes `shared = private_key × peer_public` and returns the
    /// x-coordinate of the resulting point.
    ///
    /// **Always** call [`p256_validate_key`](Self::p256_validate_key) on the
    /// peer's public key before calling this method.
    pub fn p256_shared_secret(
        &mut self,
        private_key: &[u8; KEY_SIZE],
        peer_public: &PublicKey,
    ) -> Result<SharedSecret, Error> {
        let curve = EcdsaCurveParams::nist_p256();
        let mut point = EccPoint::new(KEY_SIZE);

        self.ecc_mul(&curve, private_key, &peer_public.x, &peer_public.y, &mut point)?;

        let mut secret = SharedSecret {
            raw: [0u8; KEY_SIZE],
        };
        secret.raw.copy_from_slice(&point.x[..KEY_SIZE]);
        Ok(secret)
    }

    /// Validate that a P-256 public key lies on the curve.
    ///
    /// Returns `Ok(true)` if the point is valid, `Ok(false)` if not.
    /// Should be called on every externally-received public key.
    pub fn p256_validate_key(&mut self, public_key: &PublicKey) -> Result<bool, Error> {
        let curve = EcdsaCurveParams::nist_p256();
        self.point_check(&curve, &public_key.x, &public_key.y)
    }
}
