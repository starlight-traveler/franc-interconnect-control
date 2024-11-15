// automatically generated by the FlatBuffers compiler, do not modify
// @generated
extern crate alloc;
extern crate flatbuffers;
use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::mem;
use core::cmp::Ordering;
use self::flatbuffers::{EndianScalar, Follow};
use super::*;
// struct Unused, aligned to 4
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq)]
pub struct Unused(pub [u8; 4]);
impl Default for Unused { 
  fn default() -> Self { 
    Self([0; 4])
  }
}
impl core::fmt::Debug for Unused {
  fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
    f.debug_struct("Unused")
      .field("a", &self.a())
      .finish()
  }
}

impl flatbuffers::SimpleToVerifyInSlice for Unused {}
impl<'a> flatbuffers::Follow<'a> for Unused {
  type Inner = &'a Unused;
  #[inline]
  unsafe fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
    <&'a Unused>::follow(buf, loc)
  }
}
impl<'a> flatbuffers::Follow<'a> for &'a Unused {
  type Inner = &'a Unused;
  #[inline]
  unsafe fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
    flatbuffers::follow_cast_ref::<Unused>(buf, loc)
  }
}
impl<'b> flatbuffers::Push for Unused {
    type Output = Unused;
    #[inline]
    unsafe fn push(&self, dst: &mut [u8], _written_len: usize) {
        let src = ::core::slice::from_raw_parts(self as *const Unused as *const u8, <Self as flatbuffers::Push>::size());
        dst.copy_from_slice(src);
    }
    #[inline]
    fn alignment() -> flatbuffers::PushAlignment {
        flatbuffers::PushAlignment::new(4)
    }
}

impl<'a> flatbuffers::Verifiable for Unused {
  #[inline]
  fn run_verifier(
    v: &mut flatbuffers::Verifier, pos: usize
  ) -> Result<(), flatbuffers::InvalidFlatbuffer> {
    use self::flatbuffers::Verifiable;
    v.in_buffer::<Self>(pos)
  }
}

impl<'a> Unused {
  #[allow(clippy::too_many_arguments)]
  pub fn new(
    a: i32,
  ) -> Self {
    let mut s = Self([0; 4]);
    s.set_a(a);
    s
  }

  pub const fn get_fully_qualified_name() -> &'static str {
    "MyGame.OtherNameSpace.Unused"
  }

  pub fn a(&self) -> i32 {
    let mut mem = core::mem::MaybeUninit::<<i32 as EndianScalar>::Scalar>::uninit();
    // Safety:
    // Created from a valid Table for this object
    // Which contains a valid value in this slot
    EndianScalar::from_little_endian(unsafe {
      core::ptr::copy_nonoverlapping(
        self.0[0..].as_ptr(),
        mem.as_mut_ptr() as *mut u8,
        core::mem::size_of::<<i32 as EndianScalar>::Scalar>(),
      );
      mem.assume_init()
    })
  }

  pub fn set_a(&mut self, x: i32) {
    let x_le = x.to_little_endian();
    // Safety:
    // Created from a valid Table for this object
    // Which contains a valid value in this slot
    unsafe {
      core::ptr::copy_nonoverlapping(
        &x_le as *const _ as *const u8,
        self.0[0..].as_mut_ptr(),
        core::mem::size_of::<<i32 as EndianScalar>::Scalar>(),
      );
    }
  }

  pub fn unpack(&self) -> UnusedT {
    UnusedT {
      a: self.a(),
    }
  }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct UnusedT {
  pub a: i32,
}
impl UnusedT {
  pub fn pack(&self) -> Unused {
    Unused::new(
      self.a,
    )
  }
}
