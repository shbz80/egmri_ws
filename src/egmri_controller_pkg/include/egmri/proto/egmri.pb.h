// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: egmri.proto

#ifndef PROTOBUF_egmri_2eproto__INCLUDED
#define PROTOBUF_egmri_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace egmri {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_egmri_2eproto();
void protobuf_AssignDesc_egmri_2eproto();
void protobuf_ShutdownFile_egmri_2eproto();

class Sample;

enum SampleType {
  ACTION = 0,
  JOINT_ANGLES = 1,
  JOINT_VELOCITIES = 2,
  NOISE = 3,
  TOTAL_DATA_TYPES = 4
};
bool SampleType_IsValid(int value);
const SampleType SampleType_MIN = ACTION;
const SampleType SampleType_MAX = TOTAL_DATA_TYPES;
const int SampleType_ARRAYSIZE = SampleType_MAX + 1;

const ::google::protobuf::EnumDescriptor* SampleType_descriptor();
inline const ::std::string& SampleType_Name(SampleType value) {
  return ::google::protobuf::internal::NameOfEnum(
    SampleType_descriptor(), value);
}
inline bool SampleType_Parse(
    const ::std::string& name, SampleType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<SampleType>(
    SampleType_descriptor(), name, value);
}
enum ActuatorType {
  LEFT_ARM = 0,
  RIGHT_ARM = 1,
  BOTH = 2,
  TOTAL_ACTUATOR_TYPES = 3
};
bool ActuatorType_IsValid(int value);
const ActuatorType ActuatorType_MIN = LEFT_ARM;
const ActuatorType ActuatorType_MAX = TOTAL_ACTUATOR_TYPES;
const int ActuatorType_ARRAYSIZE = ActuatorType_MAX + 1;

const ::google::protobuf::EnumDescriptor* ActuatorType_descriptor();
inline const ::std::string& ActuatorType_Name(ActuatorType value) {
  return ::google::protobuf::internal::NameOfEnum(
    ActuatorType_descriptor(), value);
}
inline bool ActuatorType_Parse(
    const ::std::string& name, ActuatorType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<ActuatorType>(
    ActuatorType_descriptor(), name, value);
}
enum PositionControlMode {
  NO_CONTROL = 0,
  JOINT_SPACE = 1,
  JOINT_SPACE_HOLD = 2,
  TOTAL_CONTROL_MODES = 3
};
bool PositionControlMode_IsValid(int value);
const PositionControlMode PositionControlMode_MIN = NO_CONTROL;
const PositionControlMode PositionControlMode_MAX = TOTAL_CONTROL_MODES;
const int PositionControlMode_ARRAYSIZE = PositionControlMode_MAX + 1;

const ::google::protobuf::EnumDescriptor* PositionControlMode_descriptor();
inline const ::std::string& PositionControlMode_Name(PositionControlMode value) {
  return ::google::protobuf::internal::NameOfEnum(
    PositionControlMode_descriptor(), value);
}
inline bool PositionControlMode_Parse(
    const ::std::string& name, PositionControlMode* value) {
  return ::google::protobuf::internal::ParseNamedEnum<PositionControlMode>(
    PositionControlMode_descriptor(), name, value);
}
enum ControllerType {
  TF_CONTROLLER = 0,
  IMP_CONTROLLER = 1,
  TOTAL_CONTROLLER_TYPES = 2
};
bool ControllerType_IsValid(int value);
const ControllerType ControllerType_MIN = TF_CONTROLLER;
const ControllerType ControllerType_MAX = TOTAL_CONTROLLER_TYPES;
const int ControllerType_ARRAYSIZE = ControllerType_MAX + 1;

const ::google::protobuf::EnumDescriptor* ControllerType_descriptor();
inline const ::std::string& ControllerType_Name(ControllerType value) {
  return ::google::protobuf::internal::NameOfEnum(
    ControllerType_descriptor(), value);
}
inline bool ControllerType_Parse(
    const ::std::string& name, ControllerType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<ControllerType>(
    ControllerType_descriptor(), name, value);
}
// ===================================================================

class Sample : public ::google::protobuf::Message {
 public:
  Sample();
  virtual ~Sample();

  Sample(const Sample& from);

  inline Sample& operator=(const Sample& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Sample& default_instance();

  void Swap(Sample* other);

  // implements Message ----------------------------------------------

  Sample* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Sample& from);
  void MergeFrom(const Sample& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional uint32 T = 1 [default = 100];
  inline bool has_t() const;
  inline void clear_t();
  static const int kTFieldNumber = 1;
  inline ::google::protobuf::uint32 t() const;
  inline void set_t(::google::protobuf::uint32 value);

  // optional uint32 dX = 2;
  inline bool has_dx() const;
  inline void clear_dx();
  static const int kDXFieldNumber = 2;
  inline ::google::protobuf::uint32 dx() const;
  inline void set_dx(::google::protobuf::uint32 value);

  // optional uint32 dU = 3;
  inline bool has_du() const;
  inline void clear_du();
  static const int kDUFieldNumber = 3;
  inline ::google::protobuf::uint32 du() const;
  inline void set_du(::google::protobuf::uint32 value);

  // optional uint32 dO = 4;
  inline bool has_do_() const;
  inline void clear_do_();
  static const int kDOFieldNumber = 4;
  inline ::google::protobuf::uint32 do_() const;
  inline void set_do_(::google::protobuf::uint32 value);

  // repeated float X = 5 [packed = true];
  inline int x_size() const;
  inline void clear_x();
  static const int kXFieldNumber = 5;
  inline float x(int index) const;
  inline void set_x(int index, float value);
  inline void add_x(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      x() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_x();

  // repeated float U = 6 [packed = true];
  inline int u_size() const;
  inline void clear_u();
  static const int kUFieldNumber = 6;
  inline float u(int index) const;
  inline void set_u(int index, float value);
  inline void add_u(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      u() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_u();

  // repeated float obs = 7 [packed = true];
  inline int obs_size() const;
  inline void clear_obs();
  static const int kObsFieldNumber = 7;
  inline float obs(int index) const;
  inline void set_obs(int index, float value);
  inline void add_obs(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      obs() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_obs();

  // repeated float meta = 8 [packed = true];
  inline int meta_size() const;
  inline void clear_meta();
  static const int kMetaFieldNumber = 8;
  inline float meta(int index) const;
  inline void set_meta(int index, float value);
  inline void add_meta(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      meta() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_meta();

  // @@protoc_insertion_point(class_scope:egmri.Sample)
 private:
  inline void set_has_t();
  inline void clear_has_t();
  inline void set_has_dx();
  inline void clear_has_dx();
  inline void set_has_du();
  inline void clear_has_du();
  inline void set_has_do_();
  inline void clear_has_do_();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::uint32 t_;
  ::google::protobuf::uint32 dx_;
  ::google::protobuf::uint32 du_;
  ::google::protobuf::uint32 do__;
  ::google::protobuf::RepeatedField< float > x_;
  mutable int _x_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > u_;
  mutable int _u_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > obs_;
  mutable int _obs_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > meta_;
  mutable int _meta_cached_byte_size_;
  friend void  protobuf_AddDesc_egmri_2eproto();
  friend void protobuf_AssignDesc_egmri_2eproto();
  friend void protobuf_ShutdownFile_egmri_2eproto();

  void InitAsDefaultInstance();
  static Sample* default_instance_;
};
// ===================================================================


// ===================================================================

// Sample

// optional uint32 T = 1 [default = 100];
inline bool Sample::has_t() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Sample::set_has_t() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Sample::clear_has_t() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Sample::clear_t() {
  t_ = 100u;
  clear_has_t();
}
inline ::google::protobuf::uint32 Sample::t() const {
  // @@protoc_insertion_point(field_get:egmri.Sample.T)
  return t_;
}
inline void Sample::set_t(::google::protobuf::uint32 value) {
  set_has_t();
  t_ = value;
  // @@protoc_insertion_point(field_set:egmri.Sample.T)
}

// optional uint32 dX = 2;
inline bool Sample::has_dx() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Sample::set_has_dx() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Sample::clear_has_dx() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Sample::clear_dx() {
  dx_ = 0u;
  clear_has_dx();
}
inline ::google::protobuf::uint32 Sample::dx() const {
  // @@protoc_insertion_point(field_get:egmri.Sample.dX)
  return dx_;
}
inline void Sample::set_dx(::google::protobuf::uint32 value) {
  set_has_dx();
  dx_ = value;
  // @@protoc_insertion_point(field_set:egmri.Sample.dX)
}

// optional uint32 dU = 3;
inline bool Sample::has_du() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Sample::set_has_du() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Sample::clear_has_du() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Sample::clear_du() {
  du_ = 0u;
  clear_has_du();
}
inline ::google::protobuf::uint32 Sample::du() const {
  // @@protoc_insertion_point(field_get:egmri.Sample.dU)
  return du_;
}
inline void Sample::set_du(::google::protobuf::uint32 value) {
  set_has_du();
  du_ = value;
  // @@protoc_insertion_point(field_set:egmri.Sample.dU)
}

// optional uint32 dO = 4;
inline bool Sample::has_do_() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Sample::set_has_do_() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Sample::clear_has_do_() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Sample::clear_do_() {
  do__ = 0u;
  clear_has_do_();
}
inline ::google::protobuf::uint32 Sample::do_() const {
  // @@protoc_insertion_point(field_get:egmri.Sample.dO)
  return do__;
}
inline void Sample::set_do_(::google::protobuf::uint32 value) {
  set_has_do_();
  do__ = value;
  // @@protoc_insertion_point(field_set:egmri.Sample.dO)
}

// repeated float X = 5 [packed = true];
inline int Sample::x_size() const {
  return x_.size();
}
inline void Sample::clear_x() {
  x_.Clear();
}
inline float Sample::x(int index) const {
  // @@protoc_insertion_point(field_get:egmri.Sample.X)
  return x_.Get(index);
}
inline void Sample::set_x(int index, float value) {
  x_.Set(index, value);
  // @@protoc_insertion_point(field_set:egmri.Sample.X)
}
inline void Sample::add_x(float value) {
  x_.Add(value);
  // @@protoc_insertion_point(field_add:egmri.Sample.X)
}
inline const ::google::protobuf::RepeatedField< float >&
Sample::x() const {
  // @@protoc_insertion_point(field_list:egmri.Sample.X)
  return x_;
}
inline ::google::protobuf::RepeatedField< float >*
Sample::mutable_x() {
  // @@protoc_insertion_point(field_mutable_list:egmri.Sample.X)
  return &x_;
}

// repeated float U = 6 [packed = true];
inline int Sample::u_size() const {
  return u_.size();
}
inline void Sample::clear_u() {
  u_.Clear();
}
inline float Sample::u(int index) const {
  // @@protoc_insertion_point(field_get:egmri.Sample.U)
  return u_.Get(index);
}
inline void Sample::set_u(int index, float value) {
  u_.Set(index, value);
  // @@protoc_insertion_point(field_set:egmri.Sample.U)
}
inline void Sample::add_u(float value) {
  u_.Add(value);
  // @@protoc_insertion_point(field_add:egmri.Sample.U)
}
inline const ::google::protobuf::RepeatedField< float >&
Sample::u() const {
  // @@protoc_insertion_point(field_list:egmri.Sample.U)
  return u_;
}
inline ::google::protobuf::RepeatedField< float >*
Sample::mutable_u() {
  // @@protoc_insertion_point(field_mutable_list:egmri.Sample.U)
  return &u_;
}

// repeated float obs = 7 [packed = true];
inline int Sample::obs_size() const {
  return obs_.size();
}
inline void Sample::clear_obs() {
  obs_.Clear();
}
inline float Sample::obs(int index) const {
  // @@protoc_insertion_point(field_get:egmri.Sample.obs)
  return obs_.Get(index);
}
inline void Sample::set_obs(int index, float value) {
  obs_.Set(index, value);
  // @@protoc_insertion_point(field_set:egmri.Sample.obs)
}
inline void Sample::add_obs(float value) {
  obs_.Add(value);
  // @@protoc_insertion_point(field_add:egmri.Sample.obs)
}
inline const ::google::protobuf::RepeatedField< float >&
Sample::obs() const {
  // @@protoc_insertion_point(field_list:egmri.Sample.obs)
  return obs_;
}
inline ::google::protobuf::RepeatedField< float >*
Sample::mutable_obs() {
  // @@protoc_insertion_point(field_mutable_list:egmri.Sample.obs)
  return &obs_;
}

// repeated float meta = 8 [packed = true];
inline int Sample::meta_size() const {
  return meta_.size();
}
inline void Sample::clear_meta() {
  meta_.Clear();
}
inline float Sample::meta(int index) const {
  // @@protoc_insertion_point(field_get:egmri.Sample.meta)
  return meta_.Get(index);
}
inline void Sample::set_meta(int index, float value) {
  meta_.Set(index, value);
  // @@protoc_insertion_point(field_set:egmri.Sample.meta)
}
inline void Sample::add_meta(float value) {
  meta_.Add(value);
  // @@protoc_insertion_point(field_add:egmri.Sample.meta)
}
inline const ::google::protobuf::RepeatedField< float >&
Sample::meta() const {
  // @@protoc_insertion_point(field_list:egmri.Sample.meta)
  return meta_;
}
inline ::google::protobuf::RepeatedField< float >*
Sample::mutable_meta() {
  // @@protoc_insertion_point(field_mutable_list:egmri.Sample.meta)
  return &meta_;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace egmri

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::egmri::SampleType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::egmri::SampleType>() {
  return ::egmri::SampleType_descriptor();
}
template <> struct is_proto_enum< ::egmri::ActuatorType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::egmri::ActuatorType>() {
  return ::egmri::ActuatorType_descriptor();
}
template <> struct is_proto_enum< ::egmri::PositionControlMode> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::egmri::PositionControlMode>() {
  return ::egmri::PositionControlMode_descriptor();
}
template <> struct is_proto_enum< ::egmri::ControllerType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::egmri::ControllerType>() {
  return ::egmri::ControllerType_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_egmri_2eproto__INCLUDED
