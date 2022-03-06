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
    Chunk* chunk, const char* prompt, int i, bool with_constant = false) noexcept {
  auto c = chunk->get_code(i + 1);
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

void Chunk::dis(const str_t& name) {
  std::cout << "========= [" << name << "] =========" << std::endl;
  for (int i = 0; i < codes_count();)
    i = dis_ins(i);
}

int Chunk::dis_ins(int offset) {
  auto const_insN = [](Chunk* c, const str_t& s, int i, int n) -> int {
    u8_t constant = c->get_code(i + 1);
    fprintf(stdout, "%s_%-*d %4d ", s.c_str(), 15 - Xt::as_type<int>(s.size()), n, constant);
    std::cout << "`" << c->get_constant(constant) << "`" << std::endl;
    return i + 2;
  };
  auto jump_ins = [](Chunk* c, const char* s, int i, int sign) -> int {
    u16_t jump = Xt::as_type<u16_t>((c->get_code(i + 1) << 8) | c->get_code(i + 2));
    fprintf(stdout, "%-16s %4d -> %d\n", s, i, i + 3 + jump * sign);
    return i + 3;
  };

  fprintf(stdout, "%04d ", offset);
  if (offset > 0 && lines_[offset] == lines_[offset - 1])
    std::cout << "   | ";
  else
    fprintf(stdout, "%4d ", lines_[offset]);

#define COMPOUND(x)     return dis_compound(this, #x, offset)
#define COMPOUND2(x, b) return dis_compound(this, #x, offset, (b))
#define SIMPLE(x)       return dis_simple(this, #x, offset)
#define SIMPLE2(x, n)   return dis_simple(this, #x, offset, (n))

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
  case Code::JUMP: return jump_ins(this, "JUMP", offset, 1);
  case Code::JUMP_IF_FALSE: return jump_ins(this, "JUMP_IF_FALSE", offset, 1);
  case Code::LOOP: return jump_ins(this, "LOOP", offset, -1);
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
  case Code::INVOKE_8: return const_insN(this, "INVOKE_", offset, c - Code::INVOKE_0);
  case Code::SUPER_0:
  case Code::SUPER_1:
  case Code::SUPER_2:
  case Code::SUPER_3:
  case Code::SUPER_4:
  case Code::SUPER_5:
  case Code::SUPER_6:
  case Code::SUPER_7:
  case Code::SUPER_8: return const_insN(this, "SUPER_", offset, c - Code::SUPER_0);
  case Code::CLOSURE: COMPOUND2(CLOSURE, true);
  case Code::CLOSE_UPVALUE: SIMPLE(CLOSE_UPVALUE);
  case Code::RETURN: SIMPLE(RETURN);
  case Code::CLASS: COMPOUND2(CLASS, true);
  case Code::SUBCLASS: SIMPLE(SUBCLASS);
  case Code::METHOD: COMPOUND2(METHOD, true);
  default: std::cerr << "UNKNOWN CODE: " << c << std::endl; break;
  }
  return offset + 1;
}

}
