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
#include <memory>
#include <optional>
#include <ostream>
#include <unordered_map>
#include "common.hh"

namespace loxcc::bytecc {

enum class ObjType {
  STRING,
  NATIVE,
  FUNCTION,
  UPVALUE,
  CLOSURE,
  CLASS,
  INSTANCE,
  BOUND_METHOD,
};

class VM;
class Chunk;

class BaseObject : private UnCopyable {
  ObjType type_{};
  bool marked_{};
public:
  BaseObject(ObjType type) noexcept : type_(type) {}
  virtual ~BaseObject(void) {}

  inline ObjType type(void) const { return type_; }
  inline bool marked(void) const { return marked_; }
  inline void set_marked(bool marked = true) { marked_ = marked; }

  virtual str_t stringify(void) const = 0;
  virtual bool is_truthy(void) const { return true; }
  virtual void blacken(VM& vm) {}
};

class StringObject;
class NativeObject;
class FunctionObject;
class UpvalueObject;
class ClosureObject;
class ClassObject;
class InstanceObject;
class BoundMehtodObject;

enum class ValueType {
  NIL,
  BOOLEAN,
  NUMERIC,
  OBJECT,
};

class TagValue final : public Copyable {
  static constexpr u64_t kSignBit = 1llu << 63;
  static constexpr u64_t kQNaN = 0x7ffc000000000000llu;

  enum Tag {
    NIL   = 0x01,
    FALSE = 0x02,
    TRUE  = 0x03
  };

  union DoubleUnion {
    u64_t bits64;
    u32_t bits32[2];
    double num;
  };

  inline double as_d64(u64_t u64) const {
    DoubleUnion du;
    du.bits64 = u64;
    return du.num;
  }

  inline u64_t as_u64(double d64) const {
    DoubleUnion du;
    du.num = d64;
    return du.bits64;
  }

  u64_t bits_{};

  inline bool check(ObjType type) const {
    return is_object() && as_object()->type() == type;
  }
public:
  TagValue(void) noexcept : bits_(kQNaN | Tag::NIL) {}
  TagValue(nil_t) noexcept : bits_(kQNaN | Tag::NIL) {}
  TagValue(bool b) noexcept : bits_(b ? (kQNaN | Tag::TRUE) : (kQNaN | Tag::FALSE)) {}
  TagValue(i8_t x) noexcept : bits_(x) {}
  TagValue(u8_t x) noexcept : bits_(x) {}
  TagValue(i16_t x) noexcept : bits_(x) {}
  TagValue(u16_t x) noexcept : bits_(x) {}
  TagValue(i32_t x) noexcept : bits_(x) {}
  TagValue(u32_t x) noexcept : bits_(x) {}
  TagValue(i64_t x) noexcept : bits_(x) {}
  TagValue(u64_t x) noexcept : bits_(x) {}
  TagValue(float x) noexcept : bits_(as_u64(x)) {}
  TagValue(double d) noexcept : bits_(as_u64(d)) {}
  TagValue(BaseObject* o) noexcept : bits_(kSignBit | kQNaN | (u64_t)(o)) {}
  TagValue(const TagValue& r) noexcept : bits_(r.bits_) {}
  TagValue(TagValue&& r) noexcept : bits_(std::move(r.bits_)) {}

  inline TagValue& operator=(const TagValue& r) noexcept {
    if (this != &r)
      bits_ = r.bits_;
    return *this;
  }

  inline TagValue& operator=(TagValue&& r) noexcept {
    if (this != &r)
      bits_ = std::move(r.bits_);
    return *this;
  }

  inline bool operator>(const TagValue& r) const noexcept {
    return as_numeric() > r.as_numeric();
  }

  inline bool operator>=(const TagValue& r) const noexcept {
    return as_numeric() >= r.as_numeric();
  }

  inline bool operator<(const TagValue& r) const noexcept {
    return as_numeric() < r.as_numeric();
  }

  inline bool operator<=(const TagValue& r) const noexcept {
    return as_numeric() <= r.as_numeric();
  }

  inline bool operator!(void) const noexcept {
    return !is_truthy();
  }

  inline TagValue operator-(void) const noexcept {
    return -as_numeric();
  }

  inline bool operator==(const TagValue& r) const noexcept {
    return bits_ == r.bits_;
  }

  inline bool operator!=(const TagValue& r) const noexcept {
    return bits_ != r.bits_;
  }

  inline bool is_nil(void) const { return bits_ == (kQNaN | Tag::NIL); }
  inline bool is_boolean(void) const { return (bits_ & (kQNaN | Tag::FALSE)) == (kQNaN | Tag::FALSE); }
  inline bool is_numeric(void) const { return (bits_ & kQNaN) != kQNaN; }
  inline bool is_object(void) const { return (bits_ & (kQNaN | kSignBit)) == (kQNaN | kSignBit); }
  inline bool is_string(void) const { return check(ObjType::STRING); }
  inline bool is_native(void) const { return check(ObjType::NATIVE); }
  inline bool is_function(void) const { return check(ObjType::FUNCTION); }
  inline bool is_upvalue(void) const { return check(ObjType::UPVALUE); }
  inline bool is_closure(void) const { return check(ObjType::CLOSURE); }
  inline bool is_class(void) const { return check(ObjType::CLASS); }
  inline bool is_instance(void) const { return check(ObjType::INSTANCE); }
  inline bool is_bound_method(void) const { return check(ObjType::BOUND_METHOD); }

  inline bool as_boolean(void) const { return bits_ == (kQNaN | Tag::TRUE); }
  inline double as_numeric(void) const { return as_d64(bits_); }
  inline BaseObject* as_object(void) const { return (BaseObject*)(bits_ & ~(kQNaN | kSignBit)); }

  StringObject* as_string(void) const;
  const char* as_cstring(void) const;
  NativeObject* as_native(void) const;
  FunctionObject* as_function(void) const;
  UpvalueObject* as_upvalue(void) const;
  ClosureObject* as_closure(void) const;
  ClassObject* as_class(void) const;
  InstanceObject* as_instance(void) const;
  BoundMehtodObject* as_bound_method(void) const;

  bool is_truthy(void) const;
  str_t stringify(void) const;
};

class ObjValue final : public Copyable {
  ValueType type_{ValueType::NIL};
  union {
    bool boolean;
    double numeric;
    BaseObject* object;
  } as_{};

  template <typename T> inline void set_numeric(T x) {
    as_.numeric = Xt::as_type<double>(x);
  }

  inline bool check(ObjType type) const {
    return is_object() && as_.object->type() == type;
  }
public:
  ObjValue(void) noexcept {}
  ObjValue(nil_t) noexcept {}
  ObjValue(bool b) noexcept : type_(ValueType::BOOLEAN) { as_.boolean = b; }
  ObjValue(i8_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(u8_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(i16_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(u16_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(i32_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(u32_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(i64_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(u64_t x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(float x) noexcept : type_(ValueType::NUMERIC) { set_numeric(x); }
  ObjValue(double d) noexcept : type_(ValueType::NUMERIC) { as_.numeric = d; }
  ObjValue(BaseObject* o) noexcept : type_(ValueType::OBJECT) { as_.object = o; }

  ObjValue(const ObjValue& r) noexcept
    : type_(r.type_) {
    if (type_ == ValueType::OBJECT)
      as_.object = r.as_.object;
    else
      as_.numeric = r.as_.numeric;
  }

  ObjValue(ObjValue&& r) noexcept
    : type_(std::move(r.type_)) {
    if (type_ == ValueType::OBJECT)
      as_.object = std::move(r.as_.object);
    else
      as_.numeric = std::move(r.as_.numeric);
  }

  inline ObjValue& operator=(const ObjValue& r) noexcept {
    if (this != &r) {
      type_ = r.type_;
      if (type_ == ValueType::OBJECT)
        as_.object = r.as_.object;
      else
        as_.numeric = r.as_.numeric;
    }
    return *this;
  }

  inline ObjValue& operator=(ObjValue&& r) noexcept {
    if (this != &r) {
      type_ = std::move(r.type_);
      if (type_ == ValueType::OBJECT)
        as_.object = std::move(r.as_.object);
      else
        as_.numeric = std::move(r.as_.numeric);
    }
    return *this;
  }

  inline bool operator>(const ObjValue& r) const noexcept {
    return as_.numeric > r.as_.numeric;
  }

  inline bool operator>=(const ObjValue& r) const noexcept {
    return as_.numeric >= r.as_.numeric;
  }

  inline bool operator<(const ObjValue& r) const noexcept {
    return as_.numeric < r.as_.numeric;
  }

  inline bool operator<=(const ObjValue& r) const noexcept {
    return as_.numeric <= r.as_.numeric;
  }

  inline bool operator!(void) const noexcept {
    return !is_truthy();
  }

  inline ObjValue operator-(void) const noexcept {
    return -as_numeric();
  }

  inline bool is_nil(void) const { return type_ == ValueType::NIL; }
  inline bool is_boolean(void) const { return type_ == ValueType::BOOLEAN; }
  inline bool is_numeric(void) const { return type_ == ValueType::NUMERIC; }
  inline bool is_object(void) const { return type_ == ValueType::OBJECT; }
  inline bool is_string(void) const { return check(ObjType::STRING); }
  inline bool is_native(void) const { return check(ObjType::NATIVE); }
  inline bool is_function(void) const { return check(ObjType::FUNCTION); }
  inline bool is_upvalue(void) const { return check(ObjType::UPVALUE); }
  inline bool is_closure(void) const { return check(ObjType::CLOSURE); }
  inline bool is_class(void) const { return check(ObjType::CLASS); }
  inline bool is_instance(void) const { return check(ObjType::INSTANCE); }
  inline bool is_bound_method(void) const { return check(ObjType::BOUND_METHOD); }

  inline bool as_boolean(void) const { return as_.boolean; }
  inline double as_numeric(void) const { return as_.numeric; }
  inline BaseObject* as_object(void) const { return as_.object; }

  StringObject* as_string(void) const;
  const char* as_cstring(void) const;
  NativeObject* as_native(void) const;
  FunctionObject* as_function(void) const;
  UpvalueObject* as_upvalue(void) const;
  ClosureObject* as_closure(void) const;
  ClassObject* as_class(void) const;
  InstanceObject* as_instance(void) const;
  BoundMehtodObject* as_bound_method(void) const;

  bool operator==(const ObjValue& r) const;
  bool operator!=(const ObjValue& r) const;
  bool is_truthy(void) const;
  str_t stringify(void) const;
};

#if defined(NAN_TAGGING)
using Value = TagValue;
#else
using Value = ObjValue;
#endif

using NativeFn = std::function<Value (int argc, Value* args)>;

inline std::ostream& operator<<(std::ostream& out, const Value& value) {
  return out << value.stringify();
}

class StringObject final : public BaseObject {
  int size_{};
  char* data_{};
  u32_t hash_{};
public:
  StringObject(
      const char* s, int n, u32_t h, bool replace_owner = false) noexcept;
  virtual ~StringObject(void);

  inline int size(void) const { return size_; }
  inline const char* cstr(void) const { return data_; }
  inline const char* data(void) const { return data_; }
  inline u32_t hash(void) const { return hash_; }

  virtual str_t stringify(void) const override;

  static StringObject* create(VM& vm, const str_t& s);
  static StringObject* create(VM& vm, const char* s, int n);
  static StringObject* concat(VM& vm, StringObject* a, StringObject* b);
};

class NativeObject final : public BaseObject {
  NativeFn fn_{};
public:
  NativeObject(const NativeFn& fn) noexcept;
  NativeObject(NativeFn&& fn) noexcept;

  inline NativeFn fn(void) const { return fn_; }

  virtual str_t stringify(void) const override;

  static NativeObject* create(VM& vm, const NativeFn& fn);
  static NativeObject* create(VM& vm, NativeFn&& fn);
};

class FunctionObject final : public BaseObject {
  int arity_{};
  int upvalues_count_{};
  StringObject* name_{};
  Chunk* chunk_{};
public:
  FunctionObject(StringObject* name = nullptr) noexcept;
  virtual ~FunctionObject(void);

  inline int arity(void) const { return arity_; }
  inline int inc_arity(void) { return arity_++; }
  inline int upvalues_count(void) const { return upvalues_count_; }
  inline int inc_upvalues_count(void) { return upvalues_count_++; }
  inline StringObject* name(void) const { return name_; }

  inline const char* name_astr(void) const {
    return name_ != nullptr ? name_->cstr() : "<top>";
  }

  inline void set_name(StringObject* name) { name_ = name; }
  inline Chunk* chunk(void) { return chunk_; }

  virtual str_t stringify(void) const override;
  virtual void blacken(VM& vm) override;

  static FunctionObject* create(VM& vm, StringObject* name = nullptr);
};

class UpvalueObject final : public BaseObject {
  Value* value_{};
  Value closed_{};
  UpvalueObject* next_{};
public:
  UpvalueObject(Value* value, UpvalueObject* next = nullptr) noexcept;

  inline Value* value(void) const { return value_; }
  inline void set_value(Value* value) { value_ = value; }
  inline void set_value_withptr(Value* value) { value_ = value; }
  inline void set_value_withref(const Value& value) { *value_ = value; }
  inline const Value& closed(void) const { return closed_; }
  inline Value* closed_asptr(void) { return &closed_; }
  inline void set_closed(const Value& closed) { closed_ = closed; }
  inline UpvalueObject* next(void) const { return next_; }
  inline void set_next(UpvalueObject* next) { next_ = next; }

  virtual str_t stringify(void) const override;
  virtual void blacken(VM& vm) override;

  static UpvalueObject* create(
      VM& vm, Value* value, UpvalueObject* next = nullptr);
};

class ClosureObject final : public BaseObject {
  FunctionObject* fn_{};
  int upvalues_count_{};
  UpvalueObject** upvalues_{};
public:
  ClosureObject(FunctionObject* fn) noexcept;
  virtual ~ClosureObject(void);

  inline FunctionObject* fn(void) const { return fn_; }
  inline int upvalues_count(void) const { return upvalues_count_; }
  inline UpvalueObject** upvalues(void) const { return upvalues_; }
  inline UpvalueObject* get_upvalue(int i) const { return upvalues_[i]; }

  inline void set_upvalue(int i, UpvalueObject* upvalue) {
    upvalues_[i] = upvalue;
  }

  virtual str_t stringify(void) const override;
  virtual void blacken(VM& vm) override;

  static ClosureObject* create(VM& vm, FunctionObject* fn);
};

class ClassObject final : public BaseObject {
  using MethodMap = std::unordered_map<str_t, Value>;

  StringObject* name_{};
  MethodMap methods_;
public:
  ClassObject(StringObject* name) noexcept;

  inline StringObject* name(void) const { return name_; }
  inline const char* name_astr(void) const { return name_->cstr(); }

  inline void set_method(const str_t& name, const Value& method) {
    methods_[name] = method;
  }

  inline void set_method(StringObject* name, const Value& method) {
    methods_[name->cstr()] = method;
  }

  inline std::optional<Value> get_method(StringObject* name) const {
    return get_method(name->cstr());
  }

  std::optional<Value> get_method(const str_t& name) const {
    if (auto meth_iter = methods_.find(name); meth_iter != methods_.end())
      return {meth_iter->second};
    return {};
  }

  void inherit_from(ClassObject* superclass);

  virtual str_t stringify(void) const override;
  virtual void blacken(VM& vm) override;

  static ClassObject* create(VM& vm, StringObject* name);
};

class InstanceObject final : public BaseObject {
  using AttrMap = std::unordered_map<str_t, Value>;

  ClassObject* cls_{};
  AttrMap attrs_;
public:
  InstanceObject(ClassObject* cls) noexcept;

  inline ClassObject* cls(void) const { return cls_; }

  inline void set_attr(const str_t& key, const Value& val) {
    attrs_[key] = val;
  }

  inline void set_attr(StringObject* key, const Value& val) {
    attrs_[key->cstr()] = val;
  }

  inline std::optional<Value> get_attr(StringObject* key) const {
    return get_attr(key->cstr());
  }

  std::optional<Value> get_attr(const str_t& key) const {
    if (auto attr_iter = attrs_.find(key); attr_iter != attrs_.end())
      return {attr_iter->second};
    return {};
  }

  virtual str_t stringify(void) const override;
  virtual void blacken(VM& vm) override;

  static InstanceObject* create(VM& vm, ClassObject* cls);
};

class BoundMehtodObject final : public BaseObject {
  Value owner_;
  ClosureObject* method_{};
public:
  BoundMehtodObject(const Value& owner, ClosureObject* method) noexcept;

  inline const Value& owner(void) const { return owner_; }
  inline ClosureObject* method(void) const { return method_; }

  virtual str_t stringify(void) const override;
  virtual void blacken(VM& vm) override;

  static BoundMehtodObject* create(
      VM& vm, const Value& owner, ClosureObject* method);
};

}
