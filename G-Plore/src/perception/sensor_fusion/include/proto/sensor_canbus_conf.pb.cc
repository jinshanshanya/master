// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sensor_canbus_conf.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "sensor_canbus_conf.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace glb_auto_perception_sensorfusion {
class SensorCanbusConfDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<SensorCanbusConf> {
} _SensorCanbusConf_default_instance_;

namespace protobuf_sensor_5fcanbus_5fconf_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] = {
  { NULL, NULL, 0, -1, -1, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SensorCanbusConf, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SensorCanbusConf, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SensorCanbusConf, can_card_parameter_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SensorCanbusConf, enable_debug_mode_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SensorCanbusConf, enable_receiver_log_),
  0,
  1,
  2,
};

static const ::google::protobuf::internal::MigrationSchema schemas[] = {
  { 0, 8, sizeof(SensorCanbusConf)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_SensorCanbusConf_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "sensor_canbus_conf.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace

void TableStruct::Shutdown() {
  _SensorCanbusConf_default_instance_.Shutdown();
  delete file_level_metadata[0].reflection;
}

void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  ::glb_auto_perception_sensorfusion::protobuf_can_5fcard_5fparameter_2eproto::InitDefaults();
  _SensorCanbusConf_default_instance_.DefaultConstruct();
  _SensorCanbusConf_default_instance_.get_mutable()->can_card_parameter_ = const_cast< ::glb_auto_perception_sensorfusion::CANCardParameter*>(
      ::glb_auto_perception_sensorfusion::CANCardParameter::internal_default_instance());
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] = {
      "\n\030sensor_canbus_conf.proto\022 glb_auto_per"
      "ception_sensorfusion\032\030can_card_parameter"
      ".proto\"\250\001\n\020SensorCanbusConf\022N\n\022can_card_"
      "parameter\030\001 \001(\01322.glb_auto_perception_se"
      "nsorfusion.CANCardParameter\022 \n\021enable_de"
      "bug_mode\030\002 \001(\010:\005false\022\"\n\023enable_receiver"
      "_log\030\003 \001(\010:\005false"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 257);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "sensor_canbus_conf.proto", &protobuf_RegisterTypes);
  ::glb_auto_perception_sensorfusion::protobuf_can_5fcard_5fparameter_2eproto::AddDescriptors();
  ::google::protobuf::internal::OnShutdown(&TableStruct::Shutdown);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_sensor_5fcanbus_5fconf_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SensorCanbusConf::kCanCardParameterFieldNumber;
const int SensorCanbusConf::kEnableDebugModeFieldNumber;
const int SensorCanbusConf::kEnableReceiverLogFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SensorCanbusConf::SensorCanbusConf()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_sensor_5fcanbus_5fconf_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:glb_auto_perception_sensorfusion.SensorCanbusConf)
}
SensorCanbusConf::SensorCanbusConf(const SensorCanbusConf& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_can_card_parameter()) {
    can_card_parameter_ = new ::glb_auto_perception_sensorfusion::CANCardParameter(*from.can_card_parameter_);
  } else {
    can_card_parameter_ = NULL;
  }
  ::memcpy(&enable_debug_mode_, &from.enable_debug_mode_,
    reinterpret_cast<char*>(&enable_receiver_log_) -
    reinterpret_cast<char*>(&enable_debug_mode_) + sizeof(enable_receiver_log_));
  // @@protoc_insertion_point(copy_constructor:glb_auto_perception_sensorfusion.SensorCanbusConf)
}

void SensorCanbusConf::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&can_card_parameter_, 0, reinterpret_cast<char*>(&enable_receiver_log_) -
    reinterpret_cast<char*>(&can_card_parameter_) + sizeof(enable_receiver_log_));
}

SensorCanbusConf::~SensorCanbusConf() {
  // @@protoc_insertion_point(destructor:glb_auto_perception_sensorfusion.SensorCanbusConf)
  SharedDtor();
}

void SensorCanbusConf::SharedDtor() {
  if (this != internal_default_instance()) {
    delete can_card_parameter_;
  }
}

void SensorCanbusConf::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* SensorCanbusConf::descriptor() {
  protobuf_sensor_5fcanbus_5fconf_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_sensor_5fcanbus_5fconf_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const SensorCanbusConf& SensorCanbusConf::default_instance() {
  protobuf_sensor_5fcanbus_5fconf_2eproto::InitDefaults();
  return *internal_default_instance();
}

SensorCanbusConf* SensorCanbusConf::New(::google::protobuf::Arena* arena) const {
  SensorCanbusConf* n = new SensorCanbusConf;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void SensorCanbusConf::Clear() {
// @@protoc_insertion_point(message_clear_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  if (has_can_card_parameter()) {
    GOOGLE_DCHECK(can_card_parameter_ != NULL);
    can_card_parameter_->::glb_auto_perception_sensorfusion::CANCardParameter::Clear();
  }
  if (_has_bits_[0 / 32] & 6u) {
    ::memset(&enable_debug_mode_, 0, reinterpret_cast<char*>(&enable_receiver_log_) -
      reinterpret_cast<char*>(&enable_debug_mode_) + sizeof(enable_receiver_log_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool SensorCanbusConf::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .glb_auto_perception_sensorfusion.CANCardParameter can_card_parameter = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_can_card_parameter()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool enable_debug_mode = 2 [default = false];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u)) {
          set_has_enable_debug_mode();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &enable_debug_mode_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool enable_receiver_log = 3 [default = false];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u)) {
          set_has_enable_receiver_log();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &enable_receiver_log_)));
        } else {
          goto handle_unusual;
        }
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
  // @@protoc_insertion_point(parse_success:glb_auto_perception_sensorfusion.SensorCanbusConf)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:glb_auto_perception_sensorfusion.SensorCanbusConf)
  return false;
#undef DO_
}

void SensorCanbusConf::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .glb_auto_perception_sensorfusion.CANCardParameter can_card_parameter = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->can_card_parameter_, output);
  }

  // optional bool enable_debug_mode = 2 [default = false];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(2, this->enable_debug_mode(), output);
  }

  // optional bool enable_receiver_log = 3 [default = false];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(3, this->enable_receiver_log(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:glb_auto_perception_sensorfusion.SensorCanbusConf)
}

::google::protobuf::uint8* SensorCanbusConf::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .glb_auto_perception_sensorfusion.CANCardParameter can_card_parameter = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->can_card_parameter_, deterministic, target);
  }

  // optional bool enable_debug_mode = 2 [default = false];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(2, this->enable_debug_mode(), target);
  }

  // optional bool enable_receiver_log = 3 [default = false];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(3, this->enable_receiver_log(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:glb_auto_perception_sensorfusion.SensorCanbusConf)
  return target;
}

size_t SensorCanbusConf::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  if (_has_bits_[0 / 32] & 7u) {
    // optional .glb_auto_perception_sensorfusion.CANCardParameter can_card_parameter = 1;
    if (has_can_card_parameter()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->can_card_parameter_);
    }

    // optional bool enable_debug_mode = 2 [default = false];
    if (has_enable_debug_mode()) {
      total_size += 1 + 1;
    }

    // optional bool enable_receiver_log = 3 [default = false];
    if (has_enable_receiver_log()) {
      total_size += 1 + 1;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void SensorCanbusConf::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  GOOGLE_DCHECK_NE(&from, this);
  const SensorCanbusConf* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const SensorCanbusConf>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:glb_auto_perception_sensorfusion.SensorCanbusConf)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:glb_auto_perception_sensorfusion.SensorCanbusConf)
    MergeFrom(*source);
  }
}

void SensorCanbusConf::MergeFrom(const SensorCanbusConf& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_can_card_parameter()->::glb_auto_perception_sensorfusion::CANCardParameter::MergeFrom(from.can_card_parameter());
    }
    if (cached_has_bits & 0x00000002u) {
      enable_debug_mode_ = from.enable_debug_mode_;
    }
    if (cached_has_bits & 0x00000004u) {
      enable_receiver_log_ = from.enable_receiver_log_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SensorCanbusConf::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SensorCanbusConf::CopyFrom(const SensorCanbusConf& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:glb_auto_perception_sensorfusion.SensorCanbusConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SensorCanbusConf::IsInitialized() const {
  return true;
}

void SensorCanbusConf::Swap(SensorCanbusConf* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SensorCanbusConf::InternalSwap(SensorCanbusConf* other) {
  std::swap(can_card_parameter_, other->can_card_parameter_);
  std::swap(enable_debug_mode_, other->enable_debug_mode_);
  std::swap(enable_receiver_log_, other->enable_receiver_log_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata SensorCanbusConf::GetMetadata() const {
  protobuf_sensor_5fcanbus_5fconf_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_sensor_5fcanbus_5fconf_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// SensorCanbusConf

// optional .glb_auto_perception_sensorfusion.CANCardParameter can_card_parameter = 1;
bool SensorCanbusConf::has_can_card_parameter() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void SensorCanbusConf::set_has_can_card_parameter() {
  _has_bits_[0] |= 0x00000001u;
}
void SensorCanbusConf::clear_has_can_card_parameter() {
  _has_bits_[0] &= ~0x00000001u;
}
void SensorCanbusConf::clear_can_card_parameter() {
  if (can_card_parameter_ != NULL) can_card_parameter_->::glb_auto_perception_sensorfusion::CANCardParameter::Clear();
  clear_has_can_card_parameter();
}
const ::glb_auto_perception_sensorfusion::CANCardParameter& SensorCanbusConf::can_card_parameter() const {
  // @@protoc_insertion_point(field_get:glb_auto_perception_sensorfusion.SensorCanbusConf.can_card_parameter)
  return can_card_parameter_ != NULL ? *can_card_parameter_
                         : *::glb_auto_perception_sensorfusion::CANCardParameter::internal_default_instance();
}
::glb_auto_perception_sensorfusion::CANCardParameter* SensorCanbusConf::mutable_can_card_parameter() {
  set_has_can_card_parameter();
  if (can_card_parameter_ == NULL) {
    can_card_parameter_ = new ::glb_auto_perception_sensorfusion::CANCardParameter;
  }
  // @@protoc_insertion_point(field_mutable:glb_auto_perception_sensorfusion.SensorCanbusConf.can_card_parameter)
  return can_card_parameter_;
}
::glb_auto_perception_sensorfusion::CANCardParameter* SensorCanbusConf::release_can_card_parameter() {
  // @@protoc_insertion_point(field_release:glb_auto_perception_sensorfusion.SensorCanbusConf.can_card_parameter)
  clear_has_can_card_parameter();
  ::glb_auto_perception_sensorfusion::CANCardParameter* temp = can_card_parameter_;
  can_card_parameter_ = NULL;
  return temp;
}
void SensorCanbusConf::set_allocated_can_card_parameter(::glb_auto_perception_sensorfusion::CANCardParameter* can_card_parameter) {
  delete can_card_parameter_;
  can_card_parameter_ = can_card_parameter;
  if (can_card_parameter) {
    set_has_can_card_parameter();
  } else {
    clear_has_can_card_parameter();
  }
  // @@protoc_insertion_point(field_set_allocated:glb_auto_perception_sensorfusion.SensorCanbusConf.can_card_parameter)
}

// optional bool enable_debug_mode = 2 [default = false];
bool SensorCanbusConf::has_enable_debug_mode() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void SensorCanbusConf::set_has_enable_debug_mode() {
  _has_bits_[0] |= 0x00000002u;
}
void SensorCanbusConf::clear_has_enable_debug_mode() {
  _has_bits_[0] &= ~0x00000002u;
}
void SensorCanbusConf::clear_enable_debug_mode() {
  enable_debug_mode_ = false;
  clear_has_enable_debug_mode();
}
bool SensorCanbusConf::enable_debug_mode() const {
  // @@protoc_insertion_point(field_get:glb_auto_perception_sensorfusion.SensorCanbusConf.enable_debug_mode)
  return enable_debug_mode_;
}
void SensorCanbusConf::set_enable_debug_mode(bool value) {
  set_has_enable_debug_mode();
  enable_debug_mode_ = value;
  // @@protoc_insertion_point(field_set:glb_auto_perception_sensorfusion.SensorCanbusConf.enable_debug_mode)
}

// optional bool enable_receiver_log = 3 [default = false];
bool SensorCanbusConf::has_enable_receiver_log() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void SensorCanbusConf::set_has_enable_receiver_log() {
  _has_bits_[0] |= 0x00000004u;
}
void SensorCanbusConf::clear_has_enable_receiver_log() {
  _has_bits_[0] &= ~0x00000004u;
}
void SensorCanbusConf::clear_enable_receiver_log() {
  enable_receiver_log_ = false;
  clear_has_enable_receiver_log();
}
bool SensorCanbusConf::enable_receiver_log() const {
  // @@protoc_insertion_point(field_get:glb_auto_perception_sensorfusion.SensorCanbusConf.enable_receiver_log)
  return enable_receiver_log_;
}
void SensorCanbusConf::set_enable_receiver_log(bool value) {
  set_has_enable_receiver_log();
  enable_receiver_log_ = value;
  // @@protoc_insertion_point(field_set:glb_auto_perception_sensorfusion.SensorCanbusConf.enable_receiver_log)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace glb_auto_perception_sensorfusion

// @@protoc_insertion_point(global_scope)
