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
#include "lexer.hh"

namespace loxcc {

Token Lexer::next_token(void) {
  skip_whitespace();

  begpos_ = curpos_;
  if (is_end())
    return make_token(TokenKind::TK_EOF);

  char c = advance();
  if (isalpha(c))
    return make_identifier();
  if (isdigit(c))
    return make_numeric();

#define _MKTOK(c, k) case c: return make_token(TokenKind::TK_##k)
  switch (c) {
  _MKTOK('(', LPAREN);
  _MKTOK(')', RPAREN);
  _MKTOK('{', LBRACE);
  _MKTOK('}', RBRACE);
  _MKTOK(',', COMMA);
  _MKTOK('.', DOT);
  _MKTOK('-', MINUS);
  _MKTOK('+', PLUS);
  _MKTOK(';', SEMI);
  _MKTOK('/', SLASH);
  _MKTOK('*', STAR);
  case '!': return make_token(match('=') ? TokenKind::TK_BANGEQ : TokenKind::TK_BANG);
  case '=': return make_token(match('=') ? TokenKind::TK_EQEQ : TokenKind::TK_EQ);
  case '>': return make_token(match('=') ? TokenKind::TK_GTEQ : TokenKind::TK_GT);
  case '<': return make_token(match('=') ? TokenKind::TK_LTEQ : TokenKind::TK_LT);
  case '"': return make_string();
  }
#undef _MKTOK

  return make_error("unexpected charactor");
}

void Lexer::skip_whitespace(void) {
  for (;;) {
    char c = peek();
    switch (c) {
    case ' ': case '\r': case '\t': advance(); break;
    case '\n': ++lineno_; advance(); break;
    case '/':
      if (peek_next() == '/') {
        while (!is_end() && peek() != '\n')
          advance();
      }
      else {
        return;
      }
      break;
    default: return;
    }
  }
}

Token Lexer::make_identifier(void) {
  while (isalnum(peek()))
    advance();

  str_t literal = gen_literal(begpos_, curpos_);
  return make_token(get_keyword_kind(literal), literal);
}

Token Lexer::make_numeric(void) {
  while (isdigit(peek()))
    advance();

  while (peek() == '.' && isdigit(peek_next())) {
    advance();
    while (isdigit(peek()))
      advance();
  }

  if (isalpha(peek()))
    return make_error("invalid numeric or identifier");
  return make_token(TokenKind::TK_NUMERIC);
}

Token Lexer::make_string(void) {
#define _MKCHAR(x, y) case x: c = y; advance(); break

  str_t literal;
  while (!is_end() && peek() != '"') {
    char c = peek();
    switch (c) {
    case '\n': ++lineno_; break;
    case '\\':
      switch (peek_next()) {
      _MKCHAR('"', '"');
      _MKCHAR('\\', '\\');
      _MKCHAR('%', '%');
      _MKCHAR('0', '\0');
      _MKCHAR('a', '\a');
      _MKCHAR('b', '\b');
      _MKCHAR('f', '\f');
      _MKCHAR('n', '\n');
      _MKCHAR('r', '\r');
      _MKCHAR('t', '\t');
      _MKCHAR('v', '\v');
      }
      break;
    }
    literal.push_back(c);
    advance();
  }
#undef _MKCHAR

  if (is_end())
    return make_error("unterminated string");

  advance(); // closng the string "
  return make_token(TokenKind::TK_STRING, literal);
}

}
