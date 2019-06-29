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

#ifndef BYTECC_CODEF
# define BYTECC_CODEF(c)
#endif

#ifndef BYTECC_CODE
# define BYTECC_CODE(c) BYTECC_CODEF(c)
#endif

BYTECC_CODE(CONSTANT)

BYTECC_CODE(NIL)
BYTECC_CODE(TRUE)
BYTECC_CODE(FALSE)
BYTECC_CODE(POP)

BYTECC_CODE(DEF_GLOBAL)
BYTECC_CODE(GET_GLOBAL)
BYTECC_CODE(SET_GLOBAL)
BYTECC_CODE(GET_LOCAL)
BYTECC_CODE(SET_LOCAL)
BYTECC_CODE(GET_UPVALUE)
BYTECC_CODE(SET_UPVALUE)
BYTECC_CODE(GET_ATTR)
BYTECC_CODE(SET_ATTR)

BYTECC_CODE(GET_SUPER)

BYTECC_CODE(EQ)
BYTECC_CODE(NE)
BYTECC_CODE(GT)
BYTECC_CODE(GE)
BYTECC_CODE(LT)
BYTECC_CODE(LE)

BYTECC_CODE(ADD)
BYTECC_CODE(SUB)
BYTECC_CODE(MUL)
BYTECC_CODE(DIV)
BYTECC_CODE(NOT)
BYTECC_CODE(NEG)

BYTECC_CODE(PRINT)

BYTECC_CODE(JUMP)
BYTECC_CODE(JUMP_IF_FALSE)
BYTECC_CODE(LOOP)

// calls and functions
BYTECC_CODE(CALL_0)
BYTECC_CODE(CALL_1)
BYTECC_CODE(CALL_2)
BYTECC_CODE(CALL_3)
BYTECC_CODE(CALL_4)
BYTECC_CODE(CALL_5)
BYTECC_CODE(CALL_6)
BYTECC_CODE(CALL_7)
BYTECC_CODE(CALL_8)

// methods and initializers
BYTECC_CODE(INVOKE_0)
BYTECC_CODE(INVOKE_1)
BYTECC_CODE(INVOKE_2)
BYTECC_CODE(INVOKE_3)
BYTECC_CODE(INVOKE_4)
BYTECC_CODE(INVOKE_5)
BYTECC_CODE(INVOKE_6)
BYTECC_CODE(INVOKE_7)
BYTECC_CODE(INVOKE_8)

// superclasses
BYTECC_CODE(SUPER_0)
BYTECC_CODE(SUPER_1)
BYTECC_CODE(SUPER_2)
BYTECC_CODE(SUPER_3)
BYTECC_CODE(SUPER_4)
BYTECC_CODE(SUPER_5)
BYTECC_CODE(SUPER_6)
BYTECC_CODE(SUPER_7)
BYTECC_CODE(SUPER_8)

BYTECC_CODE(CLOSURE)
BYTECC_CODE(CLOSE_UPVALUE)
BYTECC_CODE(RETURN)
BYTECC_CODE(CLASS)
BYTECC_CODE(SUBCLASS)
BYTECC_CODE(METHOD)

#undef BYTECC_CODE
#undef BYTECC_CODEF