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
#include "draco/javascript/emscripten/converter_webidl_wrapper.h"

#include <fstream>
#include "draco/compression/encode.h"
#include "draco/mesh/mesh.h"

using draco::DecoderBuffer;
using draco::Mesh;
using draco::Metadata;
using draco::PointAttribute;
using draco::PointCloud;
using draco::Status;

MetadataBuilder::MetadataBuilder() {}

bool MetadataBuilder::AddStringEntry(Metadata *metadata, const char *entry_name,
                                     const char *entry_value) {
    if (!metadata)
        return false;
    const std::string name{entry_name};
    const std::string value{entry_value};
    metadata->AddEntryString(entry_name, entry_value);
    return true;
}

bool MetadataBuilder::AddIntEntry(Metadata *metadata, const char *entry_name,
                                  long entry_value) {
    if (!metadata)
        return false;
    const std::string name{entry_name};
    metadata->AddEntryInt(name, entry_value);
    return true;
}

bool MetadataBuilder::AddDoubleEntry(Metadata *metadata, const char *entry_name,
                                     double entry_value) {
    if (!metadata)
        return false;
    const std::string name{entry_name};
    metadata->AddEntryDouble(name, entry_value);
    return true;
}

int PointCloudBuilder::AddFloatAttribute(PointCloud *pc,
                                         draco_GeometryAttribute_Type type,
                                         long num_vertices, long num_components,
                                         const float *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_FLOAT32);
}

int PointCloudBuilder::AddInt8Attribute(PointCloud *pc,
                                        draco_GeometryAttribute_Type type,
                                        long num_vertices, long num_components,
                                        const char *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_INT8);
}

int PointCloudBuilder::AddUInt8Attribute(PointCloud *pc,
                                         draco_GeometryAttribute_Type type,
                                         long num_vertices, long num_components,
                                         const uint8_t *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_UINT8);
}

int PointCloudBuilder::AddInt16Attribute(PointCloud *pc,
                                         draco_GeometryAttribute_Type type,
                                         long num_vertices, long num_components,
                                         const int16_t *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_INT16);
}

int PointCloudBuilder::AddUInt16Attribute(PointCloud *pc,
                                          draco_GeometryAttribute_Type type,
                                          long num_vertices,
                                          long num_components,
                                          const uint16_t *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_UINT16);
}

int PointCloudBuilder::AddInt32Attribute(PointCloud *pc,
                                         draco_GeometryAttribute_Type type,
                                         long num_vertices, long num_components,
                                         const int32_t *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_INT32);
}

int PointCloudBuilder::AddUInt32Attribute(PointCloud *pc,
                                          draco_GeometryAttribute_Type type,
                                          long num_vertices,
                                          long num_components,
                                          const uint32_t *att_values) {
    return AddAttribute(pc, type, num_vertices, num_components, att_values,
                        draco::DT_UINT32);
}

bool PointCloudBuilder::AddMetadata(PointCloud *pc, const Metadata *metadata) {
    if (!pc)
        return false;
    // Not allow write over metadata.
    if (pc->metadata())
        return false;
    std::unique_ptr<draco::GeometryMetadata> new_metadata =
            std::unique_ptr<draco::GeometryMetadata>(
                    new draco::GeometryMetadata(*metadata));
    pc->AddMetadata(std::move(new_metadata));
    return true;
}

bool PointCloudBuilder::SetMetadataForAttribute(PointCloud *pc,
                                                long attribute_id,
                                                const Metadata *metadata) {
    if (!pc)
        return false;
    // If empty metadata, just ignore.
    if (!metadata)
        return false;
    if (attribute_id < 0 || attribute_id >= pc->num_attributes())
        return false;

    if (!pc->metadata()) {
        std::unique_ptr<draco::GeometryMetadata> geometry_metadata =
                std::unique_ptr<draco::GeometryMetadata>(new draco::GeometryMetadata());
        pc->AddMetadata(std::move(geometry_metadata));
    }

    // Get unique attribute id for the attribute.
    const long unique_id = pc->attribute(attribute_id)->unique_id();

    std::unique_ptr<draco::AttributeMetadata> att_metadata =
            std::unique_ptr<draco::AttributeMetadata>(
                    new draco::AttributeMetadata(*metadata));
    att_metadata->set_att_unique_id(unique_id);
    pc->metadata()->AddAttributeMetadata(std::move(att_metadata));
    return true;
}

MeshBuilder::MeshBuilder() {}

bool MeshBuilder::AddFacesToMesh(Mesh *mesh, long num_faces, const int *faces) {
    if (!mesh)
        return false;
    mesh->SetNumFaces(num_faces);
    for (draco::FaceIndex i(0); i < num_faces; ++i) {
        draco::Mesh::Face face;
        face[0] = faces[i.value() * 3];
        face[1] = faces[i.value() * 3 + 1];
        face[2] = faces[i.value() * 3 + 2];
        mesh->SetFace(i, face);
    }
    return true;
}

int MeshBuilder::AddFloatAttributeToMesh(Mesh *mesh,
                                         draco_GeometryAttribute_Type type,
                                         long num_vertices, long num_components,
                                         const float *att_values) {
    return AddFloatAttribute(mesh, type, num_vertices, num_components,
                             att_values);
}

int MeshBuilder::AddInt32AttributeToMesh(draco::Mesh *mesh,
                                         draco_GeometryAttribute_Type type,
                                         long num_vertices, long num_components,
                                         const int32_t *att_values) {
    return AddInt32Attribute(mesh, type, num_vertices, num_components,
                             att_values);
}

bool MeshBuilder::AddMetadataToMesh(Mesh *mesh, const Metadata *metadata) {
    return AddMetadata(mesh, metadata);
}

Encoder::Encoder() {}

void Encoder::SetEncodingMethod(long method) {
    encoder_.SetEncodingMethod(method);
}

void Encoder::SetAttributeQuantization(draco_GeometryAttribute_Type type,
                                       long quantization_bits) {
    encoder_.SetAttributeQuantization(type, quantization_bits);
}

void Encoder::SetAttributeExplicitQuantization(
        draco_GeometryAttribute_Type type, long quantization_bits,
        long num_components, const float *origin, float range) {
    encoder_.SetAttributeExplicitQuantization(type, quantization_bits,
                                              num_components, origin, range);
}

void Encoder::SetSpeedOptions(long encoding_speed, long decoding_speed) {
    encoder_.SetSpeedOptions(encoding_speed, decoding_speed);
}

void Encoder::SetTrackEncodedProperties(bool flag) {
    encoder_.SetTrackEncodedProperties(flag);
}

int Encoder::EncodeMeshToDracoBuffer(Mesh *mesh, DracoInt8Array *draco_buffer) {
    if (!mesh)
        return 0;
    draco::EncoderBuffer buffer;
    if (mesh->GetNamedAttributeId(draco::GeometryAttribute::POSITION) == -1)
        return 0;
    if (!mesh->DeduplicateAttributeValues())
        return 0;
    mesh->DeduplicatePointIds();
    if (!encoder_.EncodeMeshToBuffer(*mesh, &buffer).ok()) {
        return 0;
    }
    draco_buffer->SetValues((const signed char *) buffer.data(), buffer.size());
    return buffer.size();
}

int Encoder::EncodePointCloudToDracoBuffer(draco::PointCloud *pc,
                                           bool deduplicate_values,
                                           DracoInt8Array *draco_buffer) {
    // TODO(ostava): Refactor common functionality with EncodeMeshToDracoBuffer().
    if (!pc)
        return 0;
    draco::EncoderBuffer buffer;
    if (pc->GetNamedAttributeId(draco::GeometryAttribute::POSITION) == -1)
        return 0;
    if (deduplicate_values) {
        if (!pc->DeduplicateAttributeValues())
            return 0;
        pc->DeduplicatePointIds();
    }
    if (!encoder_.EncodePointCloudToBuffer(*pc, &buffer).ok()) {
        return 0;
    }
    draco_buffer->SetValues((const signed char *) buffer.data(), buffer.size());
    return buffer.size();
}

int Encoder::GetNumberOfEncodedPoints() {
    return encoder_.num_encoded_points();
}

int Encoder::GetNumberOfEncodedFaces() { return encoder_.num_encoded_faces(); }

ExpertEncoder::ExpertEncoder(PointCloud *pc) : pc_(pc) {
    // Web-IDL interface does not support constructor overloading so instead we
    // use RTTI to determine whether the input is a mesh or a point cloud.
    Mesh *mesh = dynamic_cast<Mesh *>(pc);
    if (mesh)
        encoder_ =
                std::unique_ptr<draco::ExpertEncoder>(new draco::ExpertEncoder(*mesh));
    else
        encoder_ =
                std::unique_ptr<draco::ExpertEncoder>(new draco::ExpertEncoder(*pc));
}

void ExpertEncoder::SetEncodingMethod(long method) {
    encoder_->SetEncodingMethod(method);
}

void ExpertEncoder::SetAttributeQuantization(long att_id,
                                             long quantization_bits) {
    encoder_->SetAttributeQuantization(att_id, quantization_bits);
}

void ExpertEncoder::SetAttributeExplicitQuantization(long att_id,
                                                     long quantization_bits,
                                                     long num_components,
                                                     const float *origin,
                                                     float range) {
    encoder_->SetAttributeExplicitQuantization(att_id, quantization_bits,
                                               num_components, origin, range);
}

void ExpertEncoder::SetSpeedOptions(long encoding_speed, long decoding_speed) {
    encoder_->SetSpeedOptions(encoding_speed, decoding_speed);
}

void ExpertEncoder::SetTrackEncodedProperties(bool flag) {
    encoder_->SetTrackEncodedProperties(flag);
}

int ExpertEncoder::EncodeToDracoBuffer(bool deduplicate_values,
                                       DracoInt8Array *draco_buffer) {
    if (!pc_)
        return 0;
    if (deduplicate_values) {
        if (!pc_->DeduplicateAttributeValues())
            return 0;
        pc_->DeduplicatePointIds();
    }

    draco::EncoderBuffer buffer;
    if (!encoder_->EncodeToBuffer(&buffer).ok()) {
        return 0;
    }
    draco_buffer->SetValues((const signed char *) buffer.data(), buffer.size());
    return buffer.size();
}

int ExpertEncoder::GetNumberOfEncodedPoints() {
    return encoder_->num_encoded_points();
}

int ExpertEncoder::GetNumberOfEncodedFaces() {
    return encoder_->num_encoded_faces();
}

MetadataQuerier::MetadataQuerier() : entry_names_metadata_(nullptr) {}

bool MetadataQuerier::HasEntry(const Metadata &metadata,
                               const char *entry_name) const {
    return metadata.entries().count(entry_name) > 0;
}

long MetadataQuerier::GetIntEntry(const Metadata &metadata,
                                  const char *entry_name) const {
    int32_t value = 0;
    const std::string name(entry_name);
    metadata.GetEntryInt(name, &value);
    return value;
}

void MetadataQuerier::GetIntEntryArray(const draco::Metadata &metadata,
                                       const char *entry_name,
                                       DracoInt32Array *out_values) const {
    const std::string name(entry_name);
    std::vector<int32_t> values;
    metadata.GetEntryIntArray(name, &values);
    out_values->MoveData(std::move(values));
}

double MetadataQuerier::GetDoubleEntry(const Metadata &metadata,
                                       const char *entry_name) const {
    double value = 0;
    const std::string name(entry_name);
    metadata.GetEntryDouble(name, &value);
    return value;
}

const char *MetadataQuerier::GetStringEntry(const Metadata &metadata,
                                            const char *entry_name) {
    const std::string name(entry_name);
    if (!metadata.GetEntryString(name, &last_string_returned_))
        return nullptr;

    const char *value = last_string_returned_.c_str();
    return value;
}

long MetadataQuerier::NumEntries(const Metadata &metadata) const {
    return metadata.num_entries();
}

const char *MetadataQuerier::GetEntryName(const Metadata &metadata,
                                          int entry_id) {
    if (entry_names_metadata_ != &metadata) {
        entry_names_.clear();
        entry_names_metadata_ = &metadata;
        // Initialize the list of entry names.
        for (auto &&entry : metadata.entries()) {
            entry_names_.push_back(entry.first);
        }
    }
    if (entry_id < 0 || entry_id >= entry_names_.size())
        return nullptr;
    return entry_names_[entry_id].c_str();
}

Decoder::Decoder() {}

draco_EncodedGeometryType Decoder::GetEncodedGeometryType(
        DecoderBuffer *in_buffer) {
    return draco::Decoder::GetEncodedGeometryType(in_buffer).value();
}

const Status *Decoder::DecodeBufferToPointCloud(DecoderBuffer *in_buffer,
                                                PointCloud *out_point_cloud) {
    last_status_ = decoder_.DecodeBufferToGeometry(in_buffer, out_point_cloud);
    return &last_status_;
}

const Status *Decoder::DecodeBufferToMesh(DecoderBuffer *in_buffer,
                                          Mesh *out_mesh) {
    last_status_ = decoder_.DecodeBufferToGeometry(in_buffer, out_mesh);
    return &last_status_;
}

long Decoder::GetAttributeId(const PointCloud &pc,
                             draco_GeometryAttribute_Type type) const {
    return pc.GetNamedAttributeId(type);
}

const PointAttribute *Decoder::GetAttribute(const PointCloud &pc, long att_id) {
    return pc.attribute(att_id);
}

const PointAttribute *Decoder::GetAttributeByUniqueId(const PointCloud &pc,
                                                      long unique_id) {
    return pc.GetAttributeByUniqueId(unique_id);
}

long Decoder::GetAttributeIdByName(const PointCloud &pc,
                                   const char *attribute_name) {
    const std::string entry_value(attribute_name);
    return pc.GetAttributeIdByMetadataEntry("name", entry_value);
}

long Decoder::GetAttributeIdByMetadataEntry(const PointCloud &pc,
                                            const char *metadata_name,
                                            const char *metadata_value) {
    const std::string entry_name(metadata_name);
    const std::string entry_value(metadata_value);
    return pc.GetAttributeIdByMetadataEntry(entry_name, entry_value);
}

bool Decoder::GetFaceFromMesh(const Mesh &m,
                              draco::FaceIndex::ValueType face_id,
                              DracoInt32Array *out_values) {
    const Mesh::Face &face = m.face(draco::FaceIndex(face_id));
    const auto ptr = reinterpret_cast<const int32_t *>(face.data());
    out_values->MoveData(std::vector<int32_t>({ptr, ptr + face.size()}));
    return true;
}

long Decoder::GetTriangleStripsFromMesh(const Mesh &m,
                                        DracoInt32Array *strip_values) {
    draco::MeshStripifier stripifier;
    std::vector<int32_t> strip_indices;
    if (!stripifier.GenerateTriangleStripsWithDegenerateTriangles(
            m, std::back_inserter(strip_indices))) {
        return 0;
    }
    strip_values->MoveData(std::move(strip_indices));
    return stripifier.num_strips();
}

bool Decoder::GetAttributeFloat(const PointAttribute &pa,
                                draco::AttributeValueIndex::ValueType val_index,
                                DracoFloat32Array *out_values) {
    const int kMaxAttributeFloatValues = 4;
    const int components = pa.num_components();
    float values[kMaxAttributeFloatValues] = {-2.0, -2.0, -2.0, -2.0};
    if (!pa.ConvertValue<float>(draco::AttributeValueIndex(val_index), values))
        return false;
    out_values->MoveData({values, values + components});
    return true;
}

bool Decoder::GetAttributeFloatForAllPoints(const PointCloud &pc,
                                            const PointAttribute &pa,
                                            DracoFloat32Array *out_values) {
    const int components = pa.num_components();
    const int num_points = pc.num_points();
    const int num_entries = num_points * components;
    const int kMaxAttributeFloatValues = 4;
    float values[kMaxAttributeFloatValues] = {-2.0, -2.0, -2.0, -2.0};
    int entry_id = 0;

    out_values->Resize(num_entries);
    for (draco::PointIndex i(0); i < num_points; ++i) {
        const draco::AttributeValueIndex val_index = pa.mapped_index(i);
        if (!pa.ConvertValue<float>(val_index, values))
            return false;
        for (int j = 0; j < components; ++j) {
            out_values->SetValue(entry_id++, values[j]);
        }
    }
    return true;
}

bool Decoder::GetAttributeInt8ForAllPoints(const PointCloud &pc,
                                           const PointAttribute &pa,
                                           DracoInt8Array *out_values) {
    return GetAttributeDataForAllPoints<DracoInt8Array, int8_t>(
            pc, pa, draco::DT_INT8, draco::DT_UINT8, out_values);
}

bool Decoder::GetAttributeUInt8ForAllPoints(const PointCloud &pc,
                                            const PointAttribute &pa,
                                            DracoUInt8Array *out_values) {
    return GetAttributeDataForAllPoints<DracoUInt8Array, uint8_t>(
            pc, pa, draco::DT_INT8, draco::DT_UINT8, out_values);
}

bool Decoder::GetAttributeInt16ForAllPoints(const PointCloud &pc,
                                            const PointAttribute &pa,
                                            DracoInt16Array *out_values) {
    return GetAttributeDataForAllPoints<DracoInt16Array, int16_t>(
            pc, pa, draco::DT_INT16, draco::DT_UINT16, out_values);
}

bool Decoder::GetAttributeUInt16ForAllPoints(const PointCloud &pc,
                                             const PointAttribute &pa,
                                             DracoUInt16Array *out_values) {
    return GetAttributeDataForAllPoints<DracoUInt16Array, uint16_t>(
            pc, pa, draco::DT_INT16, draco::DT_UINT16, out_values);
}

bool Decoder::GetAttributeInt32ForAllPoints(const PointCloud &pc,
                                            const PointAttribute &pa,
                                            DracoInt32Array *out_values) {
    return GetAttributeDataForAllPoints<DracoInt32Array, int32_t>(
            pc, pa, draco::DT_INT32, draco::DT_UINT32, out_values);
}

bool Decoder::GetAttributeIntForAllPoints(const PointCloud &pc,
                                          const PointAttribute &pa,
                                          DracoInt32Array *out_values) {
    return GetAttributeInt32ForAllPoints(pc, pa, out_values);
}

bool Decoder::GetAttributeUInt32ForAllPoints(const PointCloud &pc,
                                             const PointAttribute &pa,
                                             DracoUInt32Array *out_values) {
    return GetAttributeDataForAllPoints<DracoUInt32Array, uint32_t>(
            pc, pa, draco::DT_INT32, draco::DT_UINT32, out_values);
}

void Decoder::SkipAttributeTransform(draco_GeometryAttribute_Type att_type) {
    decoder_.SetSkipAttributeTransform(att_type);
}

const Metadata *Decoder::GetMetadata(const PointCloud &pc) const {
    return pc.GetMetadata();
}

const Metadata *Decoder::GetAttributeMetadata(const PointCloud &pc,
                                              long att_id) const {
    return pc.GetAttributeMetadataByAttributeId(att_id);
}

const draco::Status *ObjDecoder::ConvertMesh(const char *inFile, const char *outFile, EncoderOptions *options) {
    std::string inFileStr(inFile);
    std::string outFileStr(outFile);

    draco::Mesh mesh;

    decoder_.set_use_metadata(options->get_use_metadata());
    last_status_ = decoder_.DecodeFromFile(inFile, &mesh);

    if (!last_status_.ok())
        return &last_status_;

    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, options->get_pos_quantization_bits());
    encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, options->get_tex_coords_quantization_bits());
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, options->get_normals_quantization_bits());
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, options->get_generic_quantization_bits());
    const int speed = 10 - options->get_compression_level();
    encoder.SetSpeedOptions(speed, speed);

    draco::EncoderBuffer buffer;
    last_status_ = encoder.EncodeMeshToBuffer(mesh, &buffer);

    if (!last_status_.ok())
        return &last_status_;

    std::ofstream out_file(outFileStr, std::ios::binary);
    if (!out_file) {
        last_status_ = Status(Status::DRACO_ERROR, "Failed to create the output file.\n");
        return &last_status_;
    }
    out_file.write(buffer.data(), buffer.size());

    return &last_status_;
}

const draco::Status *ObjDecoder::DecodeMeshFromFile(const char *filename, draco::Mesh *out_mesh) {
    std::string filenameCnv(filename);
    last_status_ = decoder_.DecodeFromFile(filenameCnv, out_mesh);
    return &last_status_;
}

const draco::Status *ObjDecoder::DecodePointCloudFromFile(const char *filename, draco::PointCloud *out_point_cloud) {
    std::string filenameCnv(filename);
    last_status_ = decoder_.DecodeFromFile(filenameCnv, out_point_cloud);
    return &last_status_;
}

const draco::Status *ObjDecoder::DecodeMeshFromBuffer(draco::DecoderBuffer *in_buffer, draco::Mesh *out_mesh) {
    last_status_ = decoder_.DecodeFromBuffer(in_buffer, out_mesh);
    return &last_status_;
}

const draco::Status *
ObjDecoder::DecodePointCloudFromBuffer(draco::DecoderBuffer *in_buffer, draco::PointCloud *out_point_cloud) {
    last_status_ = decoder_.DecodeFromBuffer(in_buffer, out_point_cloud);
    return &last_status_;
}