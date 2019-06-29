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
#include <chrono>
#include <cstdarg>
#include <iostream>
#include "bytecc_chunk.hh"
#include "bytecc_compiler.hh"
#include "bytecc_vm.hh"

namespace loxcc::bytecc {

class CallFrame final : public Copyable {
  ClosureObject* closure_{};
  const u8_t* ip_{};
  int begpos_{};
public:
  CallFrame(ClosureObject* closure, const u8_t* ip, int begpos = 0) noexcept
    : closure_(closure), ip_(ip), begpos_(begpos) {
  }

  inline ClosureObject* closure(void) const { return closure_; }
  inline const u8_t* ip(void) const { return ip_; }
  inline int begpos(void) const { return begpos_; }

  inline void set_ip(const u8_t* ip) { ip_ = ip; }
  inline u8_t get_ip(int i) const { return ip_[i]; }
  inline u8_t inc_ip(void) { return *ip_++; }
  inline u8_t dec_ip(void) { return *ip_--; }
  inline void add_ip(int offset) { ip_ += offset; }
  inline void sub_ip(int offset) { ip_ -= offset; }

  inline FunctionObject* frame_fn(void) const { return closure_->fn(); }
  inline Chunk* frame_chunk(void) const { return closure_->fn()->chunk(); }
};

VM::VM(void) noexcept
  : gcompiler_(new GlobalCompiler()) {
  define_native("clock", [](int argc, Value* args) -> Value {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
      });
  define_native("puts", [](int argc, Value* args) -> Value {
        for (int i = 0; i < argc; ++i)
          std::cout << args[i] << " ";
        std::cout << std::endl;
        return nullptr;
      });

  ctor_string_ = StringObject::create(*this, "ctor");
}

VM::~VM(void) {
  globals_.clear();
  interned_strings_.clear();
  ctor_string_ = nullptr;

  while (!all_objects_.empty()) {
    auto* o = all_objects_.back();
    all_objects_.pop_back();
    free_object(o);
  }
  worked_objects_.clear();
}

void VM::runtime_error(const char* format, ...) {
  std::cerr << "Traceback (most recent call last):" << std::endl;
  int i = Xt::as_type<int>(frames_.size()) - 1;
  for (; i >= 0; --i) {
    auto& frame = frames_[i];
    Chunk* chunk = frame.frame_chunk();
    int ins = Xt::as_type<int>(frame.ip() - chunk->codes()) - 1;

    std::cerr << "  [LINE: " << chunk->get_line(ins) << "] in "
      << "`" << frame.frame_fn()->name_astr() << "()`" << std::endl;
  }

  va_list ap;
  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
  fprintf(stderr, "\n");

  reset();
}

void VM::reset(void) {
  stack_.clear();
  frames_.clear();
  open_upvalues_ = nullptr;
}

Value* VM::stack_values(int distance) {
  if (distance == 0 || stack_.empty())
    return nullptr;
  return &stack_[stack_.size() - distance];
}

void VM::stack_resize(int distance) {
  stack_.resize(stack_.size() - distance);
}

void VM::set_stack(int distance, const Value& v) {
  stack_[stack_.size() - distance] = v;
}

void VM::push(const Value& value) {
  stack_.push_back(value);
}

Value VM::pop(void) {
  Value value = stack_.back();
  stack_.pop_back();
  return value;
}

const Value& VM::peek(int distance) const {
  return stack_[stack_.size() - 1 - distance];
}

void VM::define_native(const str_t& name, const NativeFn& fn) {
  push(NativeObject::create(*this, fn));
  globals_[name] = peek();
  pop();
}

void VM::define_native(const str_t& name, NativeFn&& fn) {
  push(NativeObject::create(*this, std::move(fn)));
  globals_[name] = peek();
  pop();
}

void VM::define_method(StringObject* name) {
  const Value& method = peek();
  ClassObject* cls = peek(1).as_class();
  cls->set_method(name, method);
  pop();
  pop();
}

bool VM::bind_method(ClassObject* cls, StringObject* name) {
  if (auto method = cls->get_method(name); method) {
    BoundMehtodObject* bound =
      BoundMehtodObject::create(*this, peek(0), (*method).as_closure());
    pop(); // pop instance object
    push(bound);
    return true;
  }

  runtime_error("`%s` undefined attribute `%s`", cls->name_astr(), name->cstr());
  return false;
}

bool VM::call(ClosureObject* closure, int argc) {
  if (argc != closure->fn()->arity()) {
    runtime_error("`%s` takes %d arguments but %d were given",
        closure->fn()->name_astr(), closure->fn()->arity(), argc);
    return false;
  }

  if (frames_.size() >= kMaxFrames) {
    runtime_error("stack overflow");
    return false;
  }

  frames_.push_back(CallFrame(closure,
        closure->fn()->chunk()->codes(),
        Xt::as_type<int>(stack_.size() - argc - 1)));
  return true;
}

bool VM::call_value(const Value& callee, int argc) {
  if (callee.is_object()) {
    switch (callee.as_object()->type()) {
    case ObjType::NATIVE:
      {
        Value r = callee.as_native()->fn()(argc, stack_values(argc));
        stack_resize(argc + 1);
        push(r);
        return true;
      }
    case ObjType::CLOSURE:
      return call(callee.as_closure(), argc);
    case ObjType::CLASS:
      {
        ClassObject* cls = callee.as_class();
        set_stack(argc + 1, InstanceObject::create(*this, cls));
        if (auto ctor = cls->get_method(ctor_string_); ctor) {
          return call((*ctor).as_closure(), argc);
        }
        else if (argc != 0) {
          runtime_error("`%s` takes 0 arguments but %d were given",
              ctor_string_->cstr(), argc);
          return false;
        }
        return true;
      }
    case ObjType::BOUND_METHOD:
      {
        BoundMehtodObject* bound = callee.as_bound_method();
        set_stack(argc + 1, bound->owner());
        call(bound->method(), argc);
      }
    default: break; // do nothing
    }
  }

  runtime_error("can only call functions and classes");
  return false;
}

bool VM::invoke_from_class(ClassObject* cls, StringObject* name, int argc) {
  if (auto method = cls->get_method(name); method)
    return call((*method).as_closure(), argc);

  runtime_error("`%s` object has no method `%s`",
      cls->name_astr(), name->cstr());
  return false;
}

bool VM::invoke(StringObject* name, int argc) {
  const Value& owner = peek(argc);
  if (!owner.is_instance()) {
    runtime_error("only instances have methods");
    return false;
  }

  InstanceObject* inst = owner.as_instance();
  if (auto attr = inst->get_attr(name); attr) {
    set_stack(argc, (*attr));
    return call_value((*attr), argc);
  }

  return invoke_from_class(inst->cls(), name, argc);
}

UpvalueObject* VM::capture_upvalue(Value* local) {
  // if there are no open upvalues at all, need create a new one
  if (open_upvalues_ == nullptr) {
    open_upvalues_ = UpvalueObject::create(*this, local);
    return open_upvalues_;
  }

  UpvalueObject* prev_upvalue{};
  UpvalueObject* upvalue = open_upvalues_;

  // walk towards the bottom of the stack until we find a previously
  // existing upvalue or reach where it should be
  while (upvalue != nullptr && upvalue->value() > local) {
    prev_upvalue = upvalue;
    upvalue = upvalue->next();
  }
  // reuse it if we found it
  if (upvalue != nullptr && upvalue->value() == local)
    return upvalue;

  // we walked past the local on the stack, so there must not be an upvalue
  // for it already, make sure a new one and link it in the right place to
  // keep the list sorted
  UpvalueObject* created_upvalue = UpvalueObject::create(*this, local, upvalue);
  if (prev_upvalue == nullptr)
    open_upvalues_ = created_upvalue;
  else
    prev_upvalue->set_next(created_upvalue);

  return created_upvalue;
}

void VM::close_upvalues(Value* last) {
  while (open_upvalues_ != nullptr && open_upvalues_->value() >= last) {
    UpvalueObject* upvalue = open_upvalues_;

    // move the value into the upvalue itself and point the upvalue to it
    upvalue->set_closed(*upvalue->value());
    upvalue->set_value(upvalue->closed_asptr());

    // pop it off the open upvalue list
    open_upvalues_ = upvalue->next();
  }
}

InterpretRet VM::run(void) {
  CallFrame* frame = &frames_.back();

  auto _RDBYTE = [&frame](void) -> u8_t { return frame->inc_ip(); };
  auto _RDWORD = [&frame](void) -> u16_t {
    return (frame->add_ip(2),
        Xt::as_type<u16_t>((frame->get_ip(-2) << 8) | frame->get_ip(-1)));
  };
  auto _RDCONST = [&frame, _RDBYTE](void) -> const Value& {
    return frame->frame_chunk()->get_constant(_RDBYTE());
  };
  auto _RDSTRING = [_RDCONST](void) -> StringObject* {
    return _RDCONST().as_string();
  };
#define _BINARYOP(op) do {\
  if (!peek(0).is_numeric() || !peek(1).is_numeric()) {\
    runtime_error("operands must be two numerics");\
    return InterpretRet::RUNTIME_ERR;\
  }\
  double b = pop().as_numeric();\
  double a = pop().as_numeric();\
  push(a op b);\
} while (false)

#if defined(TRACE_EXEC)
# define TRACE_EXEC_INSTRUCTION() do {
    std::cout << "          ";
    for (auto& v : stack_)
      std::cout << "[" << v << "]";
    std::cout << std::endl;

    frame->frame_chunk()->dis_ins(
        Xt::as_type<int>(frame->ip() - frame->frame_chunk()->codes()));
  } while (false)
#else
# define TRACE_EXEC_INSTRUCTION() ((void)0)
#endif

#if defined(COMPUTED_GOTOS)
  static void* _dispatchs[] = {
# undef BYTECC_CODEF
# define BYTECC_CODEF(c) &&__code_##c,
# include "bytecc_codes.hh"
  };
# define INTERPRET_LOOP() DISPATCH();
# define CASE_CODE(name)  __code_##name
# define DISPATCH()       do {\
    TRACE_EXEC_INSTRUCTION();\
    goto *_dispatchs[Xt::as_type<int>(ins = Xt::as_type<Code>(_RDBYTE()))];\
  } while (false)
#else
# define INTERPRET_LOOP()\
  __loop:\
    TRACE_EXEC_INSTRUCTION();\
    switch (ins = Xt::as_type<Code>(_RDBYTE()))
# define CASE_CODE(name)  case Code::##name
# define DISPATCH()       goto __loop
#endif

  Code ins;
  INTERPRET_LOOP() {
    CASE_CODE(CONSTANT): push(_RDCONST()); DISPATCH();
    CASE_CODE(NIL): push(nullptr); DISPATCH();
    CASE_CODE(TRUE): push(true); DISPATCH();
    CASE_CODE(FALSE): push(false); DISPATCH();
    CASE_CODE(POP): pop(); DISPATCH();
    CASE_CODE(DEF_GLOBAL):
    {
      StringObject* name = _RDSTRING();
      globals_[name->cstr()] = peek(0);
      pop();

      DISPATCH();
    }
    CASE_CODE(GET_GLOBAL):
    {
      StringObject* name = _RDSTRING();
      if (auto it = globals_.find(name->cstr()); it != globals_.end()) {
        push(it->second);
      }
      else {
        runtime_error("name `%s` is not defined", name->cstr());
        return InterpretRet::RUNTIME_ERR;
      }

      DISPATCH();
    }
    CASE_CODE(SET_GLOBAL):
    {
      StringObject* name = _RDSTRING();
      if (auto it = globals_.find(name->cstr()); it == globals_.end()) {
        runtime_error("name `%s` is not defined", name->cstr());
        return InterpretRet::RUNTIME_ERR;
      }
      else {
        globals_[name->cstr()] = peek(0);
      }

      DISPATCH();
    }
    CASE_CODE(GET_LOCAL):
    {
      u8_t slot = _RDBYTE();
      push(stack_[Xt::as_type<sz_t>(frame->begpos() + slot)]);

      DISPATCH();
    }
    CASE_CODE(SET_LOCAL):
    {
      u8_t slot = _RDBYTE();
      stack_[Xt::as_type<sz_t>(frame->begpos() + slot)] = peek(0);

      DISPATCH();
    }
    CASE_CODE(GET_UPVALUE):
    {
      u8_t slot = _RDBYTE();
      push(*frame->closure()->get_upvalue(slot)->value());

      DISPATCH();
    }
    CASE_CODE(SET_UPVALUE):
    {
      u8_t slot = _RDBYTE();
      frame->closure()->get_upvalue(slot)->set_value_withref(peek(0));

      DISPATCH();
    }
    CASE_CODE(GET_ATTR):
    {
      if (!peek(0).is_instance()) {
        runtime_error("only instance objects have attributes");
        return InterpretRet::RUNTIME_ERR;
      }

      InstanceObject* inst = peek(0).as_instance();
      StringObject* name = _RDSTRING();
      if (auto attr = inst->get_attr(name); attr) {
        pop(); // pop out instance
        push(*attr);
        break;
      }
      if (!bind_method(inst->cls(), name))
        return InterpretRet::RUNTIME_ERR;

      DISPATCH();
    }
    CASE_CODE(SET_ATTR):
    {
      if (!peek(1).is_instance()) {
        runtime_error("only instance objects have attributes");
        return InterpretRet::RUNTIME_ERR;
      }

      InstanceObject* inst = peek(1).as_instance();
      inst->set_attr(_RDSTRING(), peek(0));
      Value value = pop();
      pop(); // pop instance
      push(value);

      DISPATCH();
    }
    CASE_CODE(GET_SUPER):
    {
      StringObject* name = _RDSTRING();
      ClassObject* superclass = pop().as_class();
      if (!bind_method(superclass, name))
        return InterpretRet::RUNTIME_ERR;

      DISPATCH();
    }
    CASE_CODE(EQ):
    {
      Value b = pop();
      Value a = pop();
      push(a == b);

      DISPATCH();
    }
    CASE_CODE(NE):
    {
      Value b = pop();
      Value a = pop();
      push(a != b);

      DISPATCH();
    }
    CASE_CODE(GT): _BINARYOP(>); DISPATCH();
    CASE_CODE(GE): _BINARYOP(>=); DISPATCH();
    CASE_CODE(LT): _BINARYOP(<); DISPATCH();
    CASE_CODE(LE): _BINARYOP(<=); DISPATCH();
    CASE_CODE(ADD):
    {
      if (peek(0).is_string() && peek(1).is_string()) {
        StringObject* b = peek(0).as_string();
        StringObject* a = peek(1).as_string();
        StringObject* s = StringObject::concat(*this, a, b);
        pop();
        pop();
        push(s);
      }
      else if (peek(0).is_numeric() && peek(1).is_numeric()) {
        double b = pop().as_numeric();
        double a = pop().as_numeric();
        push(a + b);
      }
      else {
        runtime_error("operands must be two strings or two numerics");
        return InterpretRet::RUNTIME_ERR;
      }

      DISPATCH();
    }
    CASE_CODE(SUB): _BINARYOP(-); DISPATCH();
    CASE_CODE(MUL): _BINARYOP(*); DISPATCH();
    CASE_CODE(DIV): _BINARYOP(/); DISPATCH();
    CASE_CODE(NOT): push(!pop()); DISPATCH();
    CASE_CODE(NEG):
    {
      if (!peek(0).is_numeric()) {
        runtime_error("operand must be a numeric");
        return InterpretRet::RUNTIME_ERR;
      }
      push(-pop());

      DISPATCH();
    }
    CASE_CODE(PRINT): std::cout << pop() << std::endl; DISPATCH();
    CASE_CODE(JUMP): frame->add_ip(_RDWORD()); DISPATCH();
    CASE_CODE(JUMP_IF_FALSE):
    {
      u16_t offset = _RDWORD();
      if (!peek(0))
        frame->add_ip(offset);

      DISPATCH();
    }
    CASE_CODE(LOOP): frame->sub_ip(_RDWORD()); DISPATCH();
    CASE_CODE(CALL_0):
    CASE_CODE(CALL_1):
    CASE_CODE(CALL_2):
    CASE_CODE(CALL_3):
    CASE_CODE(CALL_4):
    CASE_CODE(CALL_5):
    CASE_CODE(CALL_6):
    CASE_CODE(CALL_7):
    CASE_CODE(CALL_8):
    {
      int argc = ins - Code::CALL_0;
      if (!call_value(peek(argc), argc))
        return InterpretRet::RUNTIME_ERR;
      frame = &frames_.back();

      DISPATCH();
    }
    CASE_CODE(INVOKE_0):
    CASE_CODE(INVOKE_1):
    CASE_CODE(INVOKE_2):
    CASE_CODE(INVOKE_3):
    CASE_CODE(INVOKE_4):
    CASE_CODE(INVOKE_5):
    CASE_CODE(INVOKE_6):
    CASE_CODE(INVOKE_7):
    CASE_CODE(INVOKE_8):
    {
      StringObject* method_name = _RDSTRING();
      int argc = ins - Code::INVOKE_0;
      if (!invoke(method_name, argc))
        return InterpretRet::RUNTIME_ERR;
      frame = &frames_.back();

      DISPATCH();
    }
    CASE_CODE(SUPER_0):
    CASE_CODE(SUPER_1):
    CASE_CODE(SUPER_2):
    CASE_CODE(SUPER_3):
    CASE_CODE(SUPER_4):
    CASE_CODE(SUPER_5):
    CASE_CODE(SUPER_6):
    CASE_CODE(SUPER_7):
    CASE_CODE(SUPER_8):
    {
      StringObject* method_name = _RDSTRING();
      int argc = ins - Code::SUPER_0;
      ClassObject* superclass = pop().as_class();
      if (!invoke_from_class(superclass, method_name, argc))
        return InterpretRet::RUNTIME_ERR;
      frame = &frames_.back();

      DISPATCH();
    }
    CASE_CODE(CLOSURE):
    {
      // create the closure and push it on the stack before creating
      // upvalues so that is does not get collected
      FunctionObject* fn = _RDCONST().as_function();
      ClosureObject* closure = ClosureObject::create(*this, fn);
      push(closure);

      // capture upvalues
      for (int i = 0; i < closure->upvalues_count(); ++i) {
        u8_t is_local = _RDBYTE();
        u8_t index = _RDBYTE();
        if (is_local) {
          // make an new upvalue to close over the parent's local variable
          closure->set_upvalue(i, capture_upvalue(
                &stack_[Xt::as_type<int>(frame->begpos() + index)]));
        }
        else {
          // use the same upvalue as the current call frame
          closure->set_upvalue(i, frame->closure()->get_upvalue(index));
        }
      }

      DISPATCH();
    }
    CASE_CODE(CLOSE_UPVALUE): close_upvalues(&stack_.back()); pop(); DISPATCH();
    CASE_CODE(RETURN):
    {
      Value r = pop();
      if (frame->begpos() < 0 || frame->begpos() >= stack_.size())
        close_upvalues(nullptr);
      else
        close_upvalues(&stack_[frame->begpos()]);

      frames_.pop_back();
      if (frames_.empty())
        return InterpretRet::OK;

      stack_.resize(frame->begpos());
      push(r);
      frame = &frames_.back();

      DISPATCH();
    }
    CASE_CODE(CLASS): push(ClassObject::create(*this, _RDSTRING())); DISPATCH();
    CASE_CODE(SUBCLASS):
    {
      const Value& superclass = peek(1);
      if (!superclass.is_class()) {
        runtime_error("superclass must be a class");
        return InterpretRet::RUNTIME_ERR;
      }

      ClassObject* cls = peek(0).as_class();
      cls->inherit_from(superclass.as_class());
      pop();

      DISPATCH();
    }
    CASE_CODE(METHOD): define_method(_RDSTRING()); DISPATCH();
  }

#undef DISPATCH
#undef CASE_CODE
#undef INTERPRET_LOOP
#undef TRACE_EXEC_INSTRUCTION
#undef _BINARYOP
  return InterpretRet::OK;
}

void VM::collect(void) {
  // mark the stack roots
  for (auto& v : stack_)
    mark_value(v);
  for (auto& frame : frames_)
    mark_object(frame.closure());

  // mark the open upvalues
  for (auto* uv = open_upvalues_; uv != nullptr; uv = uv->next())
    mark_object(uv);

  // mark the globals roots
  for (auto& x : globals_)
    mark_value(x.second);
  gcompiler_->mark_compiler();
  mark_object(ctor_string_);

  // traverse the references
  while (!worked_objects_.empty()) {
    // pop and object from the marked worked objects list
    BaseObject* o = worked_objects_.back();
    worked_objects_.pop_back();

    o->blacken(*this);
  }

  // remove unused interned strings objects
  for (auto it = interned_strings_.begin(); it != interned_strings_.end();) {
    if (!it->second->marked())
      interned_strings_.erase(it++);
    else
      ++it;
  }

  // collect the un-marked objects
  for (auto it = all_objects_.begin(); it != all_objects_.end();) {
    if (!(*it)->marked()) {
      free_object(*it);
      all_objects_.erase(it++);
    }
    else {
      (*it)->set_marked(false);
      ++it;
    }
  }

  objects_allocated_ = all_objects_.size();
  next_gc_ = objects_allocated_ * kHeapGrowFactor;
}

void VM::append_object(BaseObject* o) {
  if (objects_allocated_ > next_gc_)
    collect();

  ++objects_allocated_;
  all_objects_.push_back(o);
}

void VM::mark_object(BaseObject* o) {
  if (o == nullptr)
    return;

  if (o->marked())
    return;

#if defined(TRACE_GC)
  std::cout << "mark object at `" << o << "` -> " << Value(o) << std::endl;
#endif

  o->set_marked(true);
  worked_objects_.push_back(o);
}

void VM::mark_value(const Value& v) {
  if (v.is_object())
    mark_object(v.as_object());
}

void VM::free_object(BaseObject* o) {
#if defined(TRACE_GC)
  std::cout << "`" << o << "` free object - " << Value(o) << std::endl;
#endif

  delete o;
}

InterpretRet VM::interpret(const str_t& source_bytes) {
  // compile the source bytes to function object code
  FunctionObject* fn = gcompiler_->compile(*this, source_bytes);
  if (fn == nullptr)
    return InterpretRet::COMPILE_ERR;

  push(fn);
  ClosureObject* closure = ClosureObject::create(*this, fn);
  pop();
  call(closure, 0);

  return run();
}

}
