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
pub enum TableInCOffset {}
#[derive(Copy, Clone, PartialEq)]

pub struct TableInC<'a> {
  pub _tab: flatbuffers::Table<'a>,
}

impl<'a> flatbuffers::Follow<'a> for TableInC<'a> {
  type Inner = TableInC<'a>;
  #[inline]
  unsafe fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
    Self { _tab: flatbuffers::Table::new(buf, loc) }
  }
}

impl<'a> TableInC<'a> {
  pub const VT_REFER_TO_A1: flatbuffers::VOffsetT = 4;
  pub const VT_REFER_TO_A2: flatbuffers::VOffsetT = 6;

  pub const fn get_fully_qualified_name() -> &'static str {
    "NamespaceC.TableInC"
  }

  #[inline]
  pub unsafe fn init_from_table(table: flatbuffers::Table<'a>) -> Self {
    TableInC { _tab: table }
  }
  #[allow(unused_mut)]
  pub fn create<'bldr: 'args, 'args: 'mut_bldr, 'mut_bldr, A: flatbuffers::Allocator + 'bldr>(
    _fbb: &'mut_bldr mut flatbuffers::FlatBufferBuilder<'bldr, A>,
    args: &'args TableInCArgs<'args>
  ) -> flatbuffers::WIPOffset<TableInC<'bldr>> {
    let mut builder = TableInCBuilder::new(_fbb);
    if let Some(x) = args.refer_to_a2 { builder.add_refer_to_a2(x); }
    if let Some(x) = args.refer_to_a1 { builder.add_refer_to_a1(x); }
    builder.finish()
  }

  pub fn unpack(&self) -> TableInCT {
    let refer_to_a1 = self.refer_to_a1().map(|x| {
      Box::new(x.unpack())
    });
    let refer_to_a2 = self.refer_to_a2().map(|x| {
      Box::new(x.unpack())
    });
    TableInCT {
      refer_to_a1,
      refer_to_a2,
    }
  }

  #[inline]
  pub fn refer_to_a1(&self) -> Option<super::namespace_a::TableInFirstNS<'a>> {
    // Safety:
    // Created from valid Table for this object
    // which contains a valid value in this slot
    unsafe { self._tab.get::<flatbuffers::ForwardsUOffset<super::namespace_a::TableInFirstNS>>(TableInC::VT_REFER_TO_A1, None)}
  }
  #[inline]
  pub fn refer_to_a2(&self) -> Option<super::namespace_a::SecondTableInA<'a>> {
    // Safety:
    // Created from valid Table for this object
    // which contains a valid value in this slot
    unsafe { self._tab.get::<flatbuffers::ForwardsUOffset<super::namespace_a::SecondTableInA>>(TableInC::VT_REFER_TO_A2, None)}
  }
}

impl flatbuffers::Verifiable for TableInC<'_> {
  #[inline]
  fn run_verifier(
    v: &mut flatbuffers::Verifier, pos: usize
  ) -> Result<(), flatbuffers::InvalidFlatbuffer> {
    use self::flatbuffers::Verifiable;
    v.visit_table(pos)?
     .visit_field::<flatbuffers::ForwardsUOffset<super::namespace_a::TableInFirstNS>>("refer_to_a1", Self::VT_REFER_TO_A1, false)?
     .visit_field::<flatbuffers::ForwardsUOffset<super::namespace_a::SecondTableInA>>("refer_to_a2", Self::VT_REFER_TO_A2, false)?
     .finish();
    Ok(())
  }
}
pub struct TableInCArgs<'a> {
    pub refer_to_a1: Option<flatbuffers::WIPOffset<super::namespace_a::TableInFirstNS<'a>>>,
    pub refer_to_a2: Option<flatbuffers::WIPOffset<super::namespace_a::SecondTableInA<'a>>>,
}
impl<'a> Default for TableInCArgs<'a> {
  #[inline]
  fn default() -> Self {
    TableInCArgs {
      refer_to_a1: None,
      refer_to_a2: None,
    }
  }
}

pub struct TableInCBuilder<'a: 'b, 'b, A: flatbuffers::Allocator + 'a> {
  fbb_: &'b mut flatbuffers::FlatBufferBuilder<'a, A>,
  start_: flatbuffers::WIPOffset<flatbuffers::TableUnfinishedWIPOffset>,
}
impl<'a: 'b, 'b, A: flatbuffers::Allocator + 'a> TableInCBuilder<'a, 'b, A> {
  #[inline]
  pub fn add_refer_to_a1(&mut self, refer_to_a1: flatbuffers::WIPOffset<super::namespace_a::TableInFirstNS<'b >>) {
    self.fbb_.push_slot_always::<flatbuffers::WIPOffset<super::namespace_a::TableInFirstNS>>(TableInC::VT_REFER_TO_A1, refer_to_a1);
  }
  #[inline]
  pub fn add_refer_to_a2(&mut self, refer_to_a2: flatbuffers::WIPOffset<super::namespace_a::SecondTableInA<'b >>) {
    self.fbb_.push_slot_always::<flatbuffers::WIPOffset<super::namespace_a::SecondTableInA>>(TableInC::VT_REFER_TO_A2, refer_to_a2);
  }
  #[inline]
  pub fn new(_fbb: &'b mut flatbuffers::FlatBufferBuilder<'a, A>) -> TableInCBuilder<'a, 'b, A> {
    let start = _fbb.start_table();
    TableInCBuilder {
      fbb_: _fbb,
      start_: start,
    }
  }
  #[inline]
  pub fn finish(self) -> flatbuffers::WIPOffset<TableInC<'a>> {
    let o = self.fbb_.end_table(self.start_);
    flatbuffers::WIPOffset::new(o.value())
  }
}

impl core::fmt::Debug for TableInC<'_> {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    let mut ds = f.debug_struct("TableInC");
      ds.field("refer_to_a1", &self.refer_to_a1());
      ds.field("refer_to_a2", &self.refer_to_a2());
      ds.finish()
  }
}
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub struct TableInCT {
  pub refer_to_a1: Option<Box<super::namespace_a::TableInFirstNST>>,
  pub refer_to_a2: Option<Box<super::namespace_a::SecondTableInAT>>,
}
impl Default for TableInCT {
  fn default() -> Self {
    Self {
      refer_to_a1: None,
      refer_to_a2: None,
    }
  }
}
impl TableInCT {
  pub fn pack<'b, A: flatbuffers::Allocator + 'b>(
    &self,
    _fbb: &mut flatbuffers::FlatBufferBuilder<'b, A>
  ) -> flatbuffers::WIPOffset<TableInC<'b>> {
    let refer_to_a1 = self.refer_to_a1.as_ref().map(|x|{
      x.pack(_fbb)
    });
    let refer_to_a2 = self.refer_to_a2.as_ref().map(|x|{
      x.pack(_fbb)
    });
    TableInC::create(_fbb, &TableInCArgs{
      refer_to_a1,
      refer_to_a2,
    })
  }
}
