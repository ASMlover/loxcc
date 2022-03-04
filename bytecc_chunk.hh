// Copyright (c) 2019 ASMlover. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list ofconditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materialsprovided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <functional>
#include <ostream>
#include <vector>
#include "common.hh"
#include "bytecc_value.hh"

namespace loxcc::bytecc {

enum class Code : u8_t {
#undef BYTECC_CODEF
#define BYTECC_CODEF(c) c,
#include "bytecc_codes.hh"
};

inline u8_t operator+(Code a, Code b) noexcept {
  return Xt::as_type<u8_t>(a) + Xt::as_type<u8_t>(b);
}

inline u8_t operator-(Code a, Code b) noexcept {
  return Xt::as_type<u8_t>(a) - Xt::as_type<u8_t>(b);
}

inline Code operator+(Code c, u8_t n) noexcept {
  return Xt::as_type<Code>(Xt::as_type<u8_t>(c) + n);
}

inline Code operator-(Code c, u8_t n) noexcept {
  return Xt::as_type<Code>(Xt::as_type<u8_t>(c) - n);
}

inline std::ostream& operator<<(std::ostream& out, Code c) noexcept {
  return out << Xt::as_type<int>(c);
}

class Chunk final : private UnCopyable {
  std::vector<u8_t> codes_;
  std::vector<int> lines_;
  std::vector<Value> constants_;
public:
  template <typename T> inline void write(T code, int lineno) noexcept {
    codes_.push_back(Xt::as_type<u8_t>(code));
    lines_.push_back(lineno);
  }

  inline u8_t add_constant(const Value& value) noexcept {
    constants_.push_back(value);
    return Xt::as_type<u8_t>(constants_.size() - 1);
  }

  inline int codes_count(void) const noexcept { return Xt::as_type<int>(codes_.size()); }
  inline const u8_t* codes(void) const noexcept { return codes_.data(); }
  inline u8_t get_code(int i) const noexcept { return codes_[i]; }
  inline void set_code(int i, u8_t c) noexcept { codes_[i] = c; }
  inline int get_line(int i) const noexcept { return lines_[i]; }
  inline int constants_count(void) const noexcept { return Xt::as_type<int>(constants_.size()); }
  inline const Value* constants(void) const noexcept { return constants_.data(); }
  inline const Value& get_constant(int i) const noexcept { return constants_[i]; }

  inline void iter_constants(std::function<void (const Value&)>&& visitor) {
    for (auto& c : constants_)
      visitor(c);
  }

  void dis(const str_t& name);
  int dis_ins(int offset);
};

}
