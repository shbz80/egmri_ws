// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: egmri.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "egmri.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace egmri {

namespace {

const ::google::protobuf::Descriptor* Sample_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Sample_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* SampleType_descriptor_ = NULL;
const ::google::protobuf::EnumDescriptor* ActuatorType_descriptor_ = NULL;
const ::google::protobuf::EnumDescriptor* PositionControlMode_descriptor_ = NULL;
const ::google::protobuf::EnumDescriptor* ControllerType_descriptor_ = NULL;

}  // namespace


void protobuf_AssignDesc_egmri_2eproto() {
  protobuf_AddDesc_egmri_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "egmri.proto");
  GOOGLE_CHECK(file != NULL);
  Sample_descriptor_ = file->message_type(0);
  static const int Sample_offsets_[8] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, t_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, dx_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, du_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, do__),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, u_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, obs_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, meta_),
  };
  Sample_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Sample_descriptor_,
      Sample::default_instance_,
      Sample_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Sample));
  SampleType_descriptor_ = file->enum_type(0);
  ActuatorType_descriptor_ = file->enum_type(1);
  PositionControlMode_descriptor_ = file->enum_type(2);
  ControllerType_descriptor_ = file->enum_type(3);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_egmri_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Sample_descriptor_, &Sample::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_egmri_2eproto() {
  delete Sample::default_instance_;
  delete Sample_reflection_;
}

void protobuf_AddDesc_egmri_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\013egmri.proto\022\005egmri\"}\n\006Sample\022\016\n\001T\030\001 \001("
    "\r:\003100\022\n\n\002dX\030\002 \001(\r\022\n\n\002dU\030\003 \001(\r\022\n\n\002dO\030\004 \001"
    "(\r\022\r\n\001X\030\005 \003(\002B\002\020\001\022\r\n\001U\030\006 \003(\002B\002\020\001\022\017\n\003obs\030"
    "\007 \003(\002B\002\020\001\022\020\n\004meta\030\010 \003(\002B\002\020\001*a\n\nSampleTyp"
    "e\022\n\n\006ACTION\020\000\022\020\n\014JOINT_ANGLES\020\001\022\024\n\020JOINT"
    "_VELOCITIES\020\002\022\t\n\005NOISE\020\003\022\024\n\020TOTAL_DATA_T"
    "YPES\020\004*O\n\014ActuatorType\022\014\n\010LEFT_ARM\020\000\022\r\n\t"
    "RIGHT_ARM\020\001\022\010\n\004BOTH\020\002\022\030\n\024TOTAL_ACTUATOR_"
    "TYPES\020\003*e\n\023PositionControlMode\022\016\n\nNO_CON"
    "TROL\020\000\022\017\n\013JOINT_SPACE\020\001\022\024\n\020JOINT_SPACE_H"
    "OLD\020\002\022\027\n\023TOTAL_CONTROL_MODES\020\003*S\n\016Contro"
    "llerType\022\021\n\rTF_CONTROLLER\020\000\022\022\n\016IMP_CONTR"
    "OLLER\020\001\022\032\n\026TOTAL_CONTROLLER_TYPES\020\002", 515);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "egmri.proto", &protobuf_RegisterTypes);
  Sample::default_instance_ = new Sample();
  Sample::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_egmri_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_egmri_2eproto {
  StaticDescriptorInitializer_egmri_2eproto() {
    protobuf_AddDesc_egmri_2eproto();
  }
} static_descriptor_initializer_egmri_2eproto_;
const ::google::protobuf::EnumDescriptor* SampleType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return SampleType_descriptor_;
}
bool SampleType_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* ActuatorType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ActuatorType_descriptor_;
}
bool ActuatorType_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* PositionControlMode_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return PositionControlMode_descriptor_;
}
bool PositionControlMode_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* ControllerType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ControllerType_descriptor_;
}
bool ControllerType_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}


// ===================================================================

#ifndef _MSC_VER
const int Sample::kTFieldNumber;
const int Sample::kDXFieldNumber;
const int Sample::kDUFieldNumber;
const int Sample::kDOFieldNumber;
const int Sample::kXFieldNumber;
const int Sample::kUFieldNumber;
const int Sample::kObsFieldNumber;
const int Sample::kMetaFieldNumber;
#endif  // !_MSC_VER

Sample::Sample()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:egmri.Sample)
}

void Sample::InitAsDefaultInstance() {
}

Sample::Sample(const Sample& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:egmri.Sample)
}

void Sample::SharedCtor() {
  _cached_size_ = 0;
  t_ = 100u;
  dx_ = 0u;
  du_ = 0u;
  do__ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Sample::~Sample() {
  // @@protoc_insertion_point(destructor:egmri.Sample)
  SharedDtor();
}

void Sample::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Sample::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Sample::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Sample_descriptor_;
}

const Sample& Sample::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_egmri_2eproto();
  return *default_instance_;
}

Sample* Sample::default_instance_ = NULL;

Sample* Sample::New() const {
  return new Sample;
}

void Sample::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<Sample*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  if (_has_bits_[0 / 32] & 15) {
    ZR_(dx_, do__);
    t_ = 100u;
  }

#undef OFFSET_OF_FIELD_
#undef ZR_

  x_.Clear();
  u_.Clear();
  obs_.Clear();
  meta_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Sample::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:egmri.Sample)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional uint32 T = 1 [default = 100];
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &t_)));
          set_has_t();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_dX;
        break;
      }

      // optional uint32 dX = 2;
      case 2: {
        if (tag == 16) {
         parse_dX:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &dx_)));
          set_has_dx();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(24)) goto parse_dU;
        break;
      }

      // optional uint32 dU = 3;
      case 3: {
        if (tag == 24) {
         parse_dU:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &du_)));
          set_has_du();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(32)) goto parse_dO;
        break;
      }

      // optional uint32 dO = 4;
      case 4: {
        if (tag == 32) {
         parse_dO:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &do__)));
          set_has_do_();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(42)) goto parse_X;
        break;
      }

      // repeated float X = 5 [packed = true];
      case 5: {
        if (tag == 42) {
         parse_X:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_x())));
        } else if (tag == 45) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 42, input, this->mutable_x())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(50)) goto parse_U;
        break;
      }

      // repeated float U = 6 [packed = true];
      case 6: {
        if (tag == 50) {
         parse_U:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_u())));
        } else if (tag == 53) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 50, input, this->mutable_u())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(58)) goto parse_obs;
        break;
      }

      // repeated float obs = 7 [packed = true];
      case 7: {
        if (tag == 58) {
         parse_obs:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_obs())));
        } else if (tag == 61) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 58, input, this->mutable_obs())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(66)) goto parse_meta;
        break;
      }

      // repeated float meta = 8 [packed = true];
      case 8: {
        if (tag == 66) {
         parse_meta:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_meta())));
        } else if (tag == 69) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 66, input, this->mutable_meta())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:egmri.Sample)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:egmri.Sample)
  return false;
#undef DO_
}

void Sample::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:egmri.Sample)
  // optional uint32 T = 1 [default = 100];
  if (has_t()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->t(), output);
  }

  // optional uint32 dX = 2;
  if (has_dx()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->dx(), output);
  }

  // optional uint32 dU = 3;
  if (has_du()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->du(), output);
  }

  // optional uint32 dO = 4;
  if (has_do_()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(4, this->do_(), output);
  }

  // repeated float X = 5 [packed = true];
  if (this->x_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(5, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_x_cached_byte_size_);
  }
  for (int i = 0; i < this->x_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->x(i), output);
  }

  // repeated float U = 6 [packed = true];
  if (this->u_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(6, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_u_cached_byte_size_);
  }
  for (int i = 0; i < this->u_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->u(i), output);
  }

  // repeated float obs = 7 [packed = true];
  if (this->obs_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(7, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_obs_cached_byte_size_);
  }
  for (int i = 0; i < this->obs_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->obs(i), output);
  }

  // repeated float meta = 8 [packed = true];
  if (this->meta_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(8, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_meta_cached_byte_size_);
  }
  for (int i = 0; i < this->meta_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->meta(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:egmri.Sample)
}

::google::protobuf::uint8* Sample::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:egmri.Sample)
  // optional uint32 T = 1 [default = 100];
  if (has_t()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->t(), target);
  }

  // optional uint32 dX = 2;
  if (has_dx()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->dx(), target);
  }

  // optional uint32 dU = 3;
  if (has_du()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(3, this->du(), target);
  }

  // optional uint32 dO = 4;
  if (has_do_()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(4, this->do_(), target);
  }

  // repeated float X = 5 [packed = true];
  if (this->x_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      5,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _x_cached_byte_size_, target);
  }
  for (int i = 0; i < this->x_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->x(i), target);
  }

  // repeated float U = 6 [packed = true];
  if (this->u_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      6,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _u_cached_byte_size_, target);
  }
  for (int i = 0; i < this->u_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->u(i), target);
  }

  // repeated float obs = 7 [packed = true];
  if (this->obs_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      7,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _obs_cached_byte_size_, target);
  }
  for (int i = 0; i < this->obs_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->obs(i), target);
  }

  // repeated float meta = 8 [packed = true];
  if (this->meta_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      8,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _meta_cached_byte_size_, target);
  }
  for (int i = 0; i < this->meta_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->meta(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:egmri.Sample)
  return target;
}

int Sample::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional uint32 T = 1 [default = 100];
    if (has_t()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->t());
    }

    // optional uint32 dX = 2;
    if (has_dx()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->dx());
    }

    // optional uint32 dU = 3;
    if (has_du()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->du());
    }

    // optional uint32 dO = 4;
    if (has_do_()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->do_());
    }

  }
  // repeated float X = 5 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->x_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _x_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float U = 6 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->u_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _u_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float obs = 7 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->obs_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _obs_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float meta = 8 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->meta_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _meta_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Sample::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Sample* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Sample*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Sample::MergeFrom(const Sample& from) {
  GOOGLE_CHECK_NE(&from, this);
  x_.MergeFrom(from.x_);
  u_.MergeFrom(from.u_);
  obs_.MergeFrom(from.obs_);
  meta_.MergeFrom(from.meta_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_t()) {
      set_t(from.t());
    }
    if (from.has_dx()) {
      set_dx(from.dx());
    }
    if (from.has_du()) {
      set_du(from.du());
    }
    if (from.has_do_()) {
      set_do_(from.do_());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Sample::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Sample::CopyFrom(const Sample& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Sample::IsInitialized() const {

  return true;
}

void Sample::Swap(Sample* other) {
  if (other != this) {
    std::swap(t_, other->t_);
    std::swap(dx_, other->dx_);
    std::swap(du_, other->du_);
    std::swap(do__, other->do__);
    x_.Swap(&other->x_);
    u_.Swap(&other->u_);
    obs_.Swap(&other->obs_);
    meta_.Swap(&other->meta_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Sample::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Sample_descriptor_;
  metadata.reflection = Sample_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace egmri

// @@protoc_insertion_point(global_scope)
