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
#include <cstring>
#include "bytecc_chunk.hh"
#include "bytecc_vm.hh"
#include "bytecc_value.hh"

namespace loxcc::bytecc {

template <typename T, typename... Args>
inline T* make_object(VM& vm, Args&&... args) {
  auto* o = new T(std::forward<Args>(args)...);
  vm.append_object(o);
  return o;
}

StringObject* TagValue::as_string(void) const {
  return Xt::down<StringObject>(as_object());
}

const char* TagValue::as_cstring(void) const {
  return Xt::down<StringObject>(as_object())->cstr();
}

NativeObject* TagValue::as_native(void) const {
  return Xt::down<NativeObject>(as_object());
}

FunctionObject* TagValue::as_function(void) const {
  return Xt::down<FunctionObject>(as_object());
}

UpvalueObject* TagValue::as_upvalue(void) const {
  return Xt::down<UpvalueObject>(as_object());
}

ClosureObject* TagValue::as_closure(void) const {
  return Xt::down<ClosureObject>(as_object());
}

ClassObject* TagValue::as_class(void) const {
  return Xt::down<ClassObject>(as_object());
}

InstanceObject* TagValue::as_instance(void) const {
  return Xt::down<InstanceObject>(as_object());
}

BoundMehtodObject* TagValue::as_bound_method(void) const {
  return Xt::down<BoundMehtodObject>(as_object());
}

bool TagValue::is_truthy(void) const {
  if (is_nil())
    return false;
  else if (is_boolean())
    return as_boolean();
  else if (is_numeric())
    return as_numeric() != 0.f;
  else if (is_object())
    return as_object()->is_truthy();
  return false;
}

str_t TagValue::stringify(void) const {
  if (is_nil())
    return "nil";
  else if (is_boolean())
    return as_boolean() ? "true" : "false";
  else if (is_numeric())
    return Xt::to_string(as_numeric());
  else if (is_object())
    return as_object()->stringify();
  return "";
}

StringObject* ObjValue::as_string(void) const {
  return Xt::down<StringObject>(as_.object);
}

const char* ObjValue::as_cstring(void) const {
  return Xt::down<StringObject>(as_.object)->cstr();
}

NativeObject* ObjValue::as_native(void) const {
  return Xt::down<NativeObject>(as_.object);
}

FunctionObject* ObjValue::as_function(void) const {
  return Xt::down<FunctionObject>(as_.object);
}

UpvalueObject* ObjValue::as_upvalue(void) const {
  return Xt::down<UpvalueObject>(as_.object);
}

ClosureObject* ObjValue::as_closure(void) const {
  return Xt::down<ClosureObject>(as_.object);
}

ClassObject* ObjValue::as_class(void) const {
  return Xt::down<ClassObject>(as_.object);
}

InstanceObject* ObjValue::as_instance(void) const {
  return Xt::down<InstanceObject>(as_.object);
}

BoundMehtodObject* ObjValue::as_bound_method(void) const {
  return Xt::down<BoundMehtodObject>(as_.object);
}

bool ObjValue::operator==(const ObjValue& r) const {
  if (this == &r)
    return true;
  if (type_ != r.type_)
    return false;

  switch (type_) {
  case ValueType::NIL: return true;
  case ValueType::BOOLEAN: return as_.boolean == r.as_.boolean;
  case ValueType::NUMERIC: return as_.numeric == r.as_.numeric;
  case ValueType::OBJECT: return as_.object == r.as_.object; // TODO: FIXME:
  }
  return false;
}

bool ObjValue::operator!=(const ObjValue& r) const {
  return !(*this == r);
}

bool ObjValue::is_truthy(void) const {
  switch (type_) {
  case ValueType::NIL: return false;
  case ValueType::BOOLEAN: return as_.boolean;
  case ValueType::NUMERIC: return as_.numeric == 0.f;
  case ValueType::OBJECT: return as_.object->is_truthy();
  }
  return false;
}

str_t ObjValue::stringify(void) const {
  switch (type_) {
  case ValueType::NIL: return "nil";
  case ValueType::BOOLEAN: return as_.boolean ? "true" : "false";
  case ValueType::NUMERIC: return Xt::to_string(as_.numeric);
  case ValueType::OBJECT: return as_.object->stringify();
  }
  return "";
}

StringObject::StringObject(
    const char* s, int n, u32_t h, bool replace_owner) noexcept
  : BaseObject(ObjType::STRING)
  , size_(n)
  , hash_(h) {
  if (replace_owner) {
    data_ = Xt::as_ptr<char>(s);
  }
  else {
    data_ = new char[Xt::as_type<sz_t>(size_) + 1];
    memcpy(data_, s, size_);
    data_[size_] = 0;
  }
}

StringObject::~StringObject(void) {
  delete [] data_;
}

str_t StringObject::stringify(void) const {
  return data_;
}

StringObject* StringObject::create(VM& vm, const str_t& s) {
  return create(vm, s.c_str(), Xt::as_type<int>(s.size()));
}

StringObject* StringObject::create(VM& vm, const char* s, int n) {
  u32_t h = Xt::hasher(s, n);
  if (auto* o = vm.get_interned(h); o != nullptr)
    return o;

  auto* o = make_object<StringObject>(vm, s, n, h);
  vm.set_interned(h, o);
  return o;
}

StringObject* StringObject::concat(VM& vm, StringObject* a, StringObject* b) {
  int n = a->size() + b->size();
  char* s = new char[Xt::as_type<sz_t>(n) + 1];
  memcpy(s, a->data(), a->size());
  memcpy(s + a->size(), b->data(), b->size());
  s[n] = 0;

  u32_t h = Xt::hasher(s, n);
  if (auto* o = vm.get_interned(h); o != nullptr) {
    delete [] s;
    return o;
  }

  auto* o = make_object<StringObject>(vm, s, n, h, true);
  vm.set_interned(h, o);
  return o;
}

NativeObject::NativeObject(const NativeFn& fn) noexcept
  : BaseObject(ObjType::NATIVE)
  , fn_(fn) {
}

NativeObject::NativeObject(NativeFn&& fn) noexcept
  : BaseObject(ObjType::NATIVE)
  , fn_(std::move(fn)) {
}

str_t NativeObject::stringify(void) const {
  std::stringstream ss;
  ss << "<native function at `" << this << "`>";
  return ss.str();
}

NativeObject* NativeObject::create(VM& vm, const NativeFn& fn) {
  return make_object<NativeObject>(vm, fn);
}

NativeObject* NativeObject::create(VM& vm, NativeFn&& fn) {
  return make_object<NativeObject>(vm, std::move(fn));
}

FunctionObject::FunctionObject(StringObject* name) noexcept
  : BaseObject(ObjType::FUNCTION)
  , name_(name)
  , chunk_(new Chunk()) {
}

FunctionObject::~FunctionObject(void) {
  delete chunk_;
}

str_t FunctionObject::stringify(void) const {
  std::stringstream ss;
  ss << "<function `" << name_astr() << "` at `" << this << "`>";
  return ss.str();
}

void FunctionObject::blacken(VM& vm) {
  vm.mark_object(name_);
  chunk_->iter_constants([&vm](const Value& v) { vm.mark_value(v); });
}

FunctionObject* FunctionObject::create(VM& vm, StringObject* name) {
  return make_object<FunctionObject>(vm, name);
}

UpvalueObject::UpvalueObject(Value* value, UpvalueObject* next) noexcept
  : BaseObject(ObjType::UPVALUE)
  , value_(value)
  , next_(next) {
}

str_t UpvalueObject::stringify(void) const {
  return "<upvalue>";
}

void UpvalueObject::blacken(VM& vm) {
  vm.mark_value(closed_);
}

UpvalueObject* UpvalueObject::create(
    VM& vm, Value* value, UpvalueObject* next) {
  return make_object<UpvalueObject>(vm, value, next);
}

ClosureObject::ClosureObject(FunctionObject* fn) noexcept
  : BaseObject(ObjType::CLOSURE)
  , fn_(fn)
  , upvalues_count_(fn->upvalues_count()) {
  if (upvalues_count_ > 0) {
    upvalues_ = new UpvalueObject*[upvalues_count_];
    for (int i = 0; i < upvalues_count_; ++i)
      upvalues_[i] = nullptr;
  }
}

ClosureObject::~ClosureObject(void) {
  if (upvalues_ != nullptr)
    delete [] upvalues_;
}

str_t ClosureObject::stringify(void) const {
  std::stringstream ss;
  ss << "<closure function `" << fn_->name_astr() << "` at `" << this << "`>";
  return ss.str();
}

void ClosureObject::blacken(VM& vm) {
  vm.mark_object(fn_);
  for (int i = 0; i < upvalues_count_; ++i)
    vm.mark_object(upvalues_[i]);
}

ClosureObject* ClosureObject::create(VM& vm, FunctionObject* fn) {
  return make_object<ClosureObject>(vm, fn);
}

ClassObject::ClassObject(StringObject* name) noexcept
  : BaseObject(ObjType::CLASS)
  , name_(name) {
}

void ClassObject::inherit_from(ClassObject* superclass) {
  for (auto& method : superclass->methods_)
    methods_[method.first] = method.second;
}

str_t ClassObject::stringify(void) const {
  std::stringstream ss;
  ss << "<class `" << name_->cstr() << "`>";
  return ss.str();
}

void ClassObject::blacken(VM& vm) {
  vm.mark_value(name_);
  for (auto& m : methods_) {
    vm.mark_value(m.second);
  }
}

ClassObject* ClassObject::create(VM& vm, StringObject* name) {
  return make_object<ClassObject>(vm, name);
}

InstanceObject::InstanceObject(ClassObject* cls) noexcept
  : BaseObject(ObjType::INSTANCE)
  , cls_(cls) {
}

str_t InstanceObject::stringify(void) const {
  std::stringstream ss;
  ss << "<`" << cls_->name_astr() << "` object at `" << this << "`>";
  return ss.str();
}

void InstanceObject::blacken(VM& vm) {
  vm.mark_object(cls_);
  for (auto& attr : attrs_)
    vm.mark_value(attr.second);
}

InstanceObject* InstanceObject::create(VM& vm, ClassObject* cls) {
  return make_object<InstanceObject>(vm, cls);
}

BoundMehtodObject::BoundMehtodObject(
    const Value& owner, ClosureObject* method) noexcept
  : BaseObject(ObjType::BOUND_METHOD)
  , owner_(owner)
  , method_(method) {
}

str_t BoundMehtodObject::stringify(void) const {
  std::stringstream ss;

  InstanceObject* inst = owner_.as_instance();
  ss << "<bound method "
    << "`" << method_->fn()->name_astr() << "` of "
    << inst->stringify() << ">";
  return ss.str();
}

void BoundMehtodObject::blacken(VM& vm) {
  vm.mark_value(owner_);
  vm.mark_object(method_);
}

BoundMehtodObject* BoundMehtodObject::create(
    VM& vm, const Value& owner, ClosureObject* method) {
  return make_object<BoundMehtodObject>(vm, owner, method);
}

}
