// Copyright 2017 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#ifndef DRACO_JAVASCRIPT_EMSCRITPEN_CONVERTER_WEBIDL_WRAPPER_H_
#define DRACO_JAVASCRIPT_EMSCRITPEN_CONVERTER_WEBIDL_WRAPPER_H_

#include <vector>

#include <emscripten.h>
#include "draco/attributes/attribute_transform_type.h"
#include "draco/attributes/point_attribute.h"
#include "draco/compression/config/compression_shared.h"
#include "draco/compression/config/encoder_options.h"
#include "draco/compression/decode.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/core/decoder_buffer.h"
#include "draco/mesh/mesh.h"
#include "draco/mesh/mesh_stripifier.h"
#include "draco/io/obj_decoder.h"

typedef draco::AttributeTransformType draco_AttributeTransformType;
typedef draco::GeometryAttribute draco_GeometryAttribute;
typedef draco::GeometryAttribute::Type draco_GeometryAttribute_Type;
typedef draco::EncodedGeometryType draco_EncodedGeometryType;
typedef draco::MeshEncoderMethod draco_MeshEncoderMethod;
typedef draco::Status draco_Status;
typedef draco::Status::Code draco_StatusCode;


// To generate Draco JavaScript bindings you must have emscripten installed.
// Then run make -f Makefile.emcc jslib.
template <typename T>
class DracoArray {
public:
    T GetValue(int index) const { return values_[index]; }

    void Resize(int size) { values_.resize(size); }
    void MoveData(std::vector<T> &&values) { values_ = std::move(values); }

    // Directly sets a value for a specific index. The array has to be already
    // allocated at this point (using Resize() method).
    void SetValue(int index, T val) { values_[index] = val; }
    bool SetValues(const T *values, int count) {
        values_.assign(values, values + count);
        return true;
    }

    int size() const { return values_.size(); }

private:
    std::vector<T> values_;
};

using DracoFloat32Array = DracoArray<float>;
using DracoInt8Array = DracoArray<int8_t>;
using DracoUInt8Array = DracoArray<uint8_t>;
using DracoInt16Array = DracoArray<int16_t>;
using DracoUInt16Array = DracoArray<uint16_t>;
using DracoInt32Array = DracoArray<int32_t>;
using DracoUInt32Array = DracoArray<uint32_t>;

class MetadataBuilder {
 public:
  MetadataBuilder();
  bool AddStringEntry(draco::Metadata *metadata, const char *entry_name,
                      const char *entry_value);
  bool AddIntEntry(draco::Metadata *metadata, const char *entry_name,
                   long entry_value);
  bool AddDoubleEntry(draco::Metadata *metadata, const char *entry_name,
                      double entry_value);
};

class PointCloudBuilder {
 public:
  PointCloudBuilder() {}
  int AddFloatAttribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const float *att_values);
  int AddInt8Attribute(draco::PointCloud *pc, draco_GeometryAttribute_Type type,
                       long num_vertices, long num_components,
                       const char *att_values);
  int AddUInt8Attribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const uint8_t *att_values);
  int AddInt16Attribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const int16_t *att_values);
  int AddUInt16Attribute(draco::PointCloud *pc,
                         draco_GeometryAttribute_Type type, long num_vertices,
                         long num_components, const uint16_t *att_values);
  int AddInt32Attribute(draco::PointCloud *pc,
                        draco_GeometryAttribute_Type type, long num_vertices,
                        long num_components, const int32_t *att_values);
  int AddUInt32Attribute(draco::PointCloud *pc,
                         draco_GeometryAttribute_Type type, long num_vertices,
                         long num_components, const uint32_t *att_values);
  bool SetMetadataForAttribute(draco::PointCloud *pc, long attribute_id,
                               const draco::Metadata *metadata);
  bool AddMetadata(draco::PointCloud *pc, const draco::Metadata *metadata);

 private:
  template <typename DataTypeT>
  int AddAttribute(draco::PointCloud *pc, draco_GeometryAttribute_Type type,
                   long num_vertices, long num_components,
                   const DataTypeT *att_values,
                   draco::DataType draco_data_type) {
    if (!pc)
      return -1;
    draco::PointAttribute att;
    att.Init(type, NULL, num_components, draco_data_type,
             /* normalized */ false,
             /* stride */ sizeof(DataTypeT) * num_components,
             /* byte_offset */ 0);
    const int att_id =
        pc->AddAttribute(att, /* identity_mapping */ true, num_vertices);
    draco::PointAttribute *const att_ptr = pc->attribute(att_id);

    for (draco::PointIndex i(0); i < num_vertices; ++i) {
      att_ptr->SetAttributeValue(att_ptr->mapped_index(i),
                                 &att_values[i.value() * num_components]);
    }
    if (pc->num_points() == 0) {
      pc->set_num_points(num_vertices);
    } else if (pc->num_points() != num_vertices) {
      return -1;
    }
    return att_id;
  }
};

// TODO(draco-eng): Regenerate wasm decoder.
// TODO(draco-eng): Add script to generate and test all Javascipt code.
class MeshBuilder : public PointCloudBuilder {
 public:
  MeshBuilder();

  bool AddFacesToMesh(draco::Mesh *mesh, long num_faces, const int *faces);

  // Deprecated: Use AddFloatAttribute() instead.
  int AddFloatAttributeToMesh(draco::Mesh *mesh,
                              draco_GeometryAttribute_Type type,
                              long num_vertices, long num_components,
                              const float *att_values);

  // Deprecated: Use AddInt32Attribute() instead.
  int AddInt32AttributeToMesh(draco::Mesh *mesh,
                              draco_GeometryAttribute_Type type,
                              long num_vertices, long num_components,
                              const int32_t *att_values);

  // Deprecated: Use AddMetadata() instead.
  bool AddMetadataToMesh(draco::Mesh *mesh, const draco::Metadata *metadata);
};

class Encoder {
 public:
  Encoder();

  void SetEncodingMethod(long method);
  void SetAttributeQuantization(draco_GeometryAttribute_Type type,
                                long quantization_bits);
  void SetAttributeExplicitQuantization(draco_GeometryAttribute_Type type,
                                        long quantization_bits,
                                        long num_components,
                                        const float *origin, float range);
  void SetSpeedOptions(long encoding_speed, long decoding_speed);
  void SetTrackEncodedProperties(bool flag);

  int EncodeMeshToDracoBuffer(draco::Mesh *mesh, DracoInt8Array *buffer);

  int EncodePointCloudToDracoBuffer(draco::PointCloud *pc,
                                    bool deduplicate_values,
                                    DracoInt8Array *buffer);
  int GetNumberOfEncodedPoints();
  int GetNumberOfEncodedFaces();

 private:
  draco::Encoder encoder_;
};

class ExpertEncoder {
 public:
  ExpertEncoder(draco::PointCloud *pc);

  void SetEncodingMethod(long method);
  void SetAttributeQuantization(long att_id, long quantization_bits);
  void SetAttributeExplicitQuantization(long att_id, long quantization_bits,
                                        long num_components,
                                        const float *origin, float range);
  void SetSpeedOptions(long encoding_speed, long decoding_speed);
  void SetTrackEncodedProperties(bool flag);

  int EncodeToDracoBuffer(bool deduplicate_values, DracoInt8Array *buffer);

  int GetNumberOfEncodedPoints();
  int GetNumberOfEncodedFaces();

 private:
  std::unique_ptr<draco::ExpertEncoder> encoder_;

  draco::PointCloud *pc_;
};

// Decoder

class MetadataQuerier {
public:
    MetadataQuerier();

    bool HasEntry(const draco::Metadata &metadata, const char *entry_name) const;

    // This function does not guarantee that entry's type is long.
    long GetIntEntry(const draco::Metadata &metadata,
                     const char *entry_name) const;

    // This function does not guarantee that entry types are long.
    void GetIntEntryArray(const draco::Metadata &metadata, const char *entry_name,
                          DracoInt32Array *out_values) const;

    // This function does not guarantee that entry's type is double.
    double GetDoubleEntry(const draco::Metadata &metadata,
                          const char *entry_name) const;

    // This function does not guarantee that entry's type is char*.
    const char *GetStringEntry(const draco::Metadata &metadata,
                               const char *entry_name);

    long NumEntries(const draco::Metadata &metadata) const;
    const char *GetEntryName(const draco::Metadata &metadata, int entry_id);

private:
    // Cached values for metadata entries.
    std::vector<std::string> entry_names_;
    const draco::Metadata *entry_names_metadata_;

    // Cached value for GetStringEntry() to avoid scoping issues.
    std::string last_string_returned_;
};

// Class used by emscripten WebIDL Binder [1] to wrap calls to decode Draco
// data.
// [1]http://kripken.github.io/emscripten-site/docs/porting/connecting_cpp_and_javascript/WebIDL-Binder.html
class Decoder {
public:
    Decoder();

    // Returns the geometry type stored in the |in_buffer|. Return values can be
    // INVALID_GEOMETRY_TYPE, POINT_CLOUD, or MESH.
    draco_EncodedGeometryType GetEncodedGeometryType(
            draco::DecoderBuffer *in_buffer);

    // Decodes a point cloud from the provided buffer.
    const draco::Status *DecodeBufferToPointCloud(
            draco::DecoderBuffer *in_buffer, draco::PointCloud *out_point_cloud);

    // Decodes a triangular mesh from the provided buffer.
    const draco::Status *DecodeBufferToMesh(draco::DecoderBuffer *in_buffer,
                                            draco::Mesh *out_mesh);

    // Returns an attribute id for the first attribute of a given type.
    long GetAttributeId(const draco::PointCloud &pc,
                        draco_GeometryAttribute_Type type) const;

    // Returns an attribute id of an attribute that contains a valid metadata
    // entry "name" with value |attribute_name|.
    static long GetAttributeIdByName(const draco::PointCloud &pc,
                                     const char *attribute_name);

    // Returns an attribute id of an attribute with a specified metadata pair
    // <|metadata_name|, |metadata_value|>.
    static long GetAttributeIdByMetadataEntry(const draco::PointCloud &pc,
                                              const char *metadata_name,
                                              const char *metadata_value);

    // Returns an attribute id of an attribute that has the unique id.
    static const draco::PointAttribute *GetAttributeByUniqueId(
            const draco::PointCloud &pc, long unique_id);

    // Returns a PointAttribute pointer from |att_id| index.
    static const draco::PointAttribute *GetAttribute(const draco::PointCloud &pc,
                                                     long att_id);

    // Returns Mesh::Face values in |out_values| from |face_id| index.
    static bool GetFaceFromMesh(const draco::Mesh &m,
                                draco::FaceIndex::ValueType face_id,
                                DracoInt32Array *out_values);

    // Returns triangle strips for mesh |m|. If there's multiple strips,
    // the strips will be separated by degenerate faces.
    static long GetTriangleStripsFromMesh(const draco::Mesh &m,
                                          DracoInt32Array *strip_values);

    // Returns float attribute values in |out_values| from |entry_index| index.
    static bool GetAttributeFloat(
            const draco::PointAttribute &pa,
            draco::AttributeValueIndex::ValueType entry_index,
            DracoFloat32Array *out_values);

    // Returns float attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeFloatForAllPoints(const draco::PointCloud &pc,
                                              const draco::PointAttribute &pa,
                                              DracoFloat32Array *out_values);

    // Returns int8_t attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeInt8ForAllPoints(const draco::PointCloud &pc,
                                             const draco::PointAttribute &pa,
                                             DracoInt8Array *out_values);

    // Returns uint8_t attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeUInt8ForAllPoints(const draco::PointCloud &pc,
                                              const draco::PointAttribute &pa,
                                              DracoUInt8Array *out_values);

    // Returns int16_t attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeInt16ForAllPoints(const draco::PointCloud &pc,
                                              const draco::PointAttribute &pa,
                                              DracoInt16Array *out_values);

    // Returns uint16_t attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeUInt16ForAllPoints(const draco::PointCloud &pc,
                                               const draco::PointAttribute &pa,
                                               DracoUInt16Array *out_values);

    // Returns int32_t attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeInt32ForAllPoints(const draco::PointCloud &pc,
                                              const draco::PointAttribute &pa,
                                              DracoInt32Array *out_values);

    // Deprecated: Use GetAttributeInt32ForAllPoints() instead.
    static bool GetAttributeIntForAllPoints(const draco::PointCloud &pc,
                                            const draco::PointAttribute &pa,
                                            DracoInt32Array *out_values);

    // Returns uint32_t attribute values for all point ids of the point cloud.
    // I.e., the |out_values| is going to contain m.num_points() entries.
    static bool GetAttributeUInt32ForAllPoints(const draco::PointCloud &pc,
                                               const draco::PointAttribute &pa,
                                               DracoUInt32Array *out_values);

    // Tells the decoder to skip an attribute transform (e.g. dequantization) for
    // an attribute of a given type.
    void SkipAttributeTransform(draco_GeometryAttribute_Type att_type);

    const draco::Metadata *GetMetadata(const draco::PointCloud &pc) const;
    const draco::Metadata *GetAttributeMetadata(const draco::PointCloud &pc,
                                                long att_id) const;

private:
    template <class DracoArrayT, class ValueTypeT>
    static bool GetAttributeDataForAllPoints(const draco::PointCloud &pc,
                                             const draco::PointAttribute &pa,
                                             draco::DataType draco_signed_type,
                                             draco::DataType draco_unsigned_type,
                                             DracoArrayT *out_values) {
        const int components = pa.num_components();
        const int num_points = pc.num_points();
        const int num_entries = num_points * components;

        if ((pa.data_type() == draco_signed_type ||
             pa.data_type() == draco_unsigned_type) &&
            pa.is_mapping_identity()) {
            // Copy values directly to the output vector.
            const auto ptr = pa.GetAddress(draco::AttributeValueIndex(0));
            out_values->MoveData({ptr, ptr + num_entries});
            return true;
        }

        // Copy values one by one.
        std::vector<ValueTypeT> values(components);
        int entry_id = 0;

        out_values->Resize(num_entries);
        for (draco::PointIndex i(0); i < num_points; ++i) {
            const draco::AttributeValueIndex val_index = pa.mapped_index(i);
            if (!pa.ConvertValue<ValueTypeT>(val_index, &values[0]))
                return false;
            for (int j = 0; j < components; ++j) {
                out_values->SetValue(entry_id++, values[j]);
            }
        }
        return true;
    }

    draco::Decoder decoder_;
    draco::Status last_status_;
};

class ObjDecoder {
public:
    ObjDecoder() {}

    const draco::Status *DecodeMeshFromBuffer(draco::DecoderBuffer *buffer, draco::Mesh *out_mesh);
    const draco::Status *DecodePointCloudFromBuffer(draco::DecoderBuffer *buffer, draco::PointCloud *out_point_cloud);

    void set_deduplicate_input_values(bool v) { decoder_.set_deduplicate_input_values(v); }
    void set_use_metadata(bool flag) { decoder_.set_use_metadata(flag); }
private:
    draco::ObjDecoder decoder_;
    draco::Status last_status_;
};

class FileHelper {
public:
    FileHelper() {}

    float WriteFile(draco::DecoderBuffer *buffer, const char *filename);
    float ReadFile(const char *filename, DracoUInt8Array *out_values);
    bool RemoveFile(const char *filename);
};

#endif  // DRACO_JAVASCRIPT_EMSCRITPEN_CONVERTER_WEBIDL_WRAPPER_H_
