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
#include <iostream>
#include "bytecc_chunk.hh"

namespace loxcc::bytecc {

inline int dis_compound(
    Chunk* chunk, const char* prompt, int i, bool with_constant = false, int n = 0) noexcept {
  auto c = chunk->get_code(i + 1);
  if (n > 1)
    std::fprintf(stdout, "%s_%-*d %4d", prompt, 15 - Xt::as_type<int>(std::strlen(prompt)), n, c);
  else
    std::fprintf(stdout, "%-16s %4d", prompt, c);
  if (with_constant)
    std::cout << " `" << chunk->get_constant(c) << "`";
  std::cout << std::endl;

  return i + 2;
}

inline int dis_simple(Chunk* chunk, const char* prompt, int i, int n = 0) noexcept {
  std::cout << prompt;
  if (n > 0)
    std::cout << "_" << n;
  std::cout << std::endl;

  return i + 1;
}

inline int dis_jump(Chunk* chunk, const char* prompt, int i, int sign) noexcept {
  u16_t jump = Xt::as_type<u16_t>((chunk->get_code(i + 1) << 8) | chunk->get_code(i + 2));
  std::fprintf(stdout, "%-16s %4d -> %d\n", prompt, i, i + 3 + jump * sign);
  return i + 3;
}

void Chunk::dis(const str_t& name) {
  std::cout << "========= [" << name << "] =========" << std::endl;
  for (int i = 0; i < codes_count();)
    i = dis_ins(i);
}

int Chunk::dis_ins(int offset) {
  fprintf(stdout, "%04d ", offset);
  if (offset > 0 && lines_[offset] == lines_[offset - 1])
    std::cout << "   | ";
  else
    fprintf(stdout, "%4d ", lines_[offset]);

#define COMPOUND(x)     return dis_compound(this, #x, offset)
#define COMPOUND2(x, b) return dis_compound(this, #x, offset, (b))
#define COMPOUND3(x, n) return dis_compound(this, #x, offset, true, (n))
#define SIMPLE(x)       return dis_simple(this, #x, offset)
#define SIMPLE2(x, n)   return dis_simple(this, #x, offset, (n))
#define JUMP(x, s)      return dis_jump(this, #x, offset, (s))

  switch (Code c = Xt::as_type<Code>(codes_[offset])) {
  case Code::CONSTANT: COMPOUND2(CONSTANT, true);
  case Code::NIL: SIMPLE(NIL);
  case Code::TRUE: SIMPLE(TRUE);
  case Code::FALSE: SIMPLE(FALSE);
  case Code::POP: SIMPLE(POP);
  case Code::DEF_GLOBAL: COMPOUND2(DEF_GLOBAL, true);
  case Code::GET_GLOBAL: COMPOUND2(GET_GLOBAL, true);
  case Code::SET_GLOBAL: COMPOUND2(SET_GLOBAL, true);
  case Code::GET_LOCAL: COMPOUND(GET_LOCAL);
  case Code::SET_LOCAL: COMPOUND(SET_LOCAL);
  case Code::GET_UPVALUE: COMPOUND(GET_UPVALUE);
  case Code::SET_UPVALUE: COMPOUND(SET_UPVALUE);
  case Code::GET_ATTR: COMPOUND2(GET_ATTR, true);
  case Code::SET_ATTR: COMPOUND2(SET_ATTR, true);
  case Code::GET_SUPER: COMPOUND2(GET_SUPER, true);
  case Code::EQ: SIMPLE(EQ);
  case Code::NE: SIMPLE(NE);
  case Code::GT: SIMPLE(GT);
  case Code::GE: SIMPLE(GE);
  case Code::LT: SIMPLE(LT);
  case Code::LE: SIMPLE(LE);
  case Code::ADD: SIMPLE(ADD);
  case Code::SUB: SIMPLE(SUB);
  case Code::MUL: SIMPLE(MUL);
  case Code::DIV: SIMPLE(DIV);
  case Code::NOT: SIMPLE(NOT);
  case Code::NEG: SIMPLE(NEG);
  case Code::PRINT: SIMPLE(PRINT);
  case Code::JUMP: JUMP(JUMP, 1);
  case Code::JUMP_IF_FALSE: JUMP(JUMP_IF_FALSE, 1);
  case Code::LOOP: JUMP(LOOP, -1);
  case Code::CALL_0:
  case Code::CALL_1:
  case Code::CALL_2:
  case Code::CALL_3:
  case Code::CALL_4:
  case Code::CALL_5:
  case Code::CALL_6:
  case Code::CALL_7:
  case Code::CALL_8: SIMPLE2(CALL, c - Code::CALL_0);
  case Code::INVOKE_0:
  case Code::INVOKE_1:
  case Code::INVOKE_2:
  case Code::INVOKE_3:
  case Code::INVOKE_4:
  case Code::INVOKE_5:
  case Code::INVOKE_6:
  case Code::INVOKE_7:
  case Code::INVOKE_8: COMPOUND3(INVOKE_, c - Code::INVOKE_0);
  case Code::SUPER_0:
  case Code::SUPER_1:
  case Code::SUPER_2:
  case Code::SUPER_3:
  case Code::SUPER_4:
  case Code::SUPER_5:
  case Code::SUPER_6:
  case Code::SUPER_7:
  case Code::SUPER_8: COMPOUND3(SUPER_, c - Code::SUPER_0);
  case Code::CLOSURE: COMPOUND2(CLOSURE, true);
  case Code::CLOSE_UPVALUE: SIMPLE(CLOSE_UPVALUE);
  case Code::RETURN: SIMPLE(RETURN);
  case Code::CLASS: COMPOUND2(CLASS, true);
  case Code::SUBCLASS: SIMPLE(SUBCLASS);
  case Code::METHOD: COMPOUND2(METHOD, true);
  default: std::cerr << "UNKNOWN CODE: " << c << std::endl; break;
  }

#undef JUMP
#undef SIMPLE2
#undef SIMPLE
#undef COMPOUND3
#undef COMPOUND2
#undef COMPOUND

  return offset + 1;
}

}
