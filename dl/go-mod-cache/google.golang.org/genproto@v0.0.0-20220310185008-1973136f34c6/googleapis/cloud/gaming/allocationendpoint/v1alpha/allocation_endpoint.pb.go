// Copyright 2021 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.26.0
// 	protoc        v3.12.2
// source: google/cloud/gaming/allocationendpoint/v1alpha/allocation_endpoint.proto

package allocationendpoint

import (
	context "context"
	reflect "reflect"
	sync "sync"

	_ "google.golang.org/genproto/googleapis/api/annotations"
	grpc "google.golang.org/grpc"
	codes "google.golang.org/grpc/codes"
	status "google.golang.org/grpc/status"
	protoreflect "google.golang.org/protobuf/reflect/protoreflect"
	protoimpl "google.golang.org/protobuf/runtime/protoimpl"
)

const (
	// Verify that this generated code is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(20 - protoimpl.MinVersion)
	// Verify that runtime/protoimpl is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(protoimpl.MaxVersion - 20)
)

type AllocationRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// The required realm name in the following form:
	// `{location}/{realm}`.
	Realm string `protobuf:"bytes,1,opt,name=realm,proto3" json:"realm,omitempty"`
	// The default game server deployment name.
	// This is used to increase the likelihood of a successful
	// allocation.
	DefaultGameServerDeployment string `protobuf:"bytes,2,opt,name=default_game_server_deployment,json=defaultGameServerDeployment,proto3" json:"default_game_server_deployment,omitempty"`
	// The ordered list of game server labels to match for allocations.
	// If the first game server selector is not matched, the selection attempts
	// the second game server selector, and so on.
	GameServerSelectors []*GameServerSelector `protobuf:"bytes,3,rep,name=game_server_selectors,json=gameServerSelectors,proto3" json:"game_server_selectors,omitempty"`
	// Metadata is optional custom metadata that is added to the game server at
	// allocation. You can use this to tell the server necessary session data.
	Metadata *MetaPatch `protobuf:"bytes,4,opt,name=metadata,proto3" json:"metadata,omitempty"`
}

func (x *AllocationRequest) Reset() {
	*x = AllocationRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *AllocationRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*AllocationRequest) ProtoMessage() {}

func (x *AllocationRequest) ProtoReflect() protoreflect.Message {
	mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use AllocationRequest.ProtoReflect.Descriptor instead.
func (*AllocationRequest) Descriptor() ([]byte, []int) {
	return file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescGZIP(), []int{0}
}

func (x *AllocationRequest) GetRealm() string {
	if x != nil {
		return x.Realm
	}
	return ""
}

func (x *AllocationRequest) GetDefaultGameServerDeployment() string {
	if x != nil {
		return x.DefaultGameServerDeployment
	}
	return ""
}

func (x *AllocationRequest) GetGameServerSelectors() []*GameServerSelector {
	if x != nil {
		return x.GameServerSelectors
	}
	return nil
}

func (x *AllocationRequest) GetMetadata() *MetaPatch {
	if x != nil {
		return x.Metadata
	}
	return nil
}

type AllocationResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// The name of the allocated game server.
	GameServerName string `protobuf:"bytes,1,opt,name=game_server_name,json=gameServerName,proto3" json:"game_server_name,omitempty"`
	// The allocated game server's port information.
	Ports []*AllocationResponse_GameServerStatusPort `protobuf:"bytes,2,rep,name=ports,proto3" json:"ports,omitempty"`
	// The address of the allocated game server.
	Address string `protobuf:"bytes,3,opt,name=address,proto3" json:"address,omitempty"`
	// The node name of the allocated game server.
	NodeName string `protobuf:"bytes,4,opt,name=node_name,json=nodeName,proto3" json:"node_name,omitempty"`
	// The game server cluster from which the game server was allocated.
	GameServerClusterName string `protobuf:"bytes,5,opt,name=game_server_cluster_name,json=gameServerClusterName,proto3" json:"game_server_cluster_name,omitempty"`
	// The game server deployment from which the game server was allocated.
	DeploymentName string `protobuf:"bytes,6,opt,name=deployment_name,json=deploymentName,proto3" json:"deployment_name,omitempty"`
}

func (x *AllocationResponse) Reset() {
	*x = AllocationResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *AllocationResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*AllocationResponse) ProtoMessage() {}

func (x *AllocationResponse) ProtoReflect() protoreflect.Message {
	mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use AllocationResponse.ProtoReflect.Descriptor instead.
func (*AllocationResponse) Descriptor() ([]byte, []int) {
	return file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescGZIP(), []int{1}
}

func (x *AllocationResponse) GetGameServerName() string {
	if x != nil {
		return x.GameServerName
	}
	return ""
}

func (x *AllocationResponse) GetPorts() []*AllocationResponse_GameServerStatusPort {
	if x != nil {
		return x.Ports
	}
	return nil
}

func (x *AllocationResponse) GetAddress() string {
	if x != nil {
		return x.Address
	}
	return ""
}

func (x *AllocationResponse) GetNodeName() string {
	if x != nil {
		return x.NodeName
	}
	return ""
}

func (x *AllocationResponse) GetGameServerClusterName() string {
	if x != nil {
		return x.GameServerClusterName
	}
	return ""
}

func (x *AllocationResponse) GetDeploymentName() string {
	if x != nil {
		return x.DeploymentName
	}
	return ""
}

// MetaPatch is the metadata used to patch the Game Server metadata on
// allocation. It behaves exactly as it does in OSS.
type MetaPatch struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Labels      map[string]string `protobuf:"bytes,1,rep,name=labels,proto3" json:"labels,omitempty" protobuf_key:"bytes,1,opt,name=key,proto3" protobuf_val:"bytes,2,opt,name=value,proto3"`
	Annotations map[string]string `protobuf:"bytes,2,rep,name=annotations,proto3" json:"annotations,omitempty" protobuf_key:"bytes,1,opt,name=key,proto3" protobuf_val:"bytes,2,opt,name=value,proto3"`
}

func (x *MetaPatch) Reset() {
	*x = MetaPatch{}
	if protoimpl.UnsafeEnabled {
		mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[2]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *MetaPatch) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*MetaPatch) ProtoMessage() {}

func (x *MetaPatch) ProtoReflect() protoreflect.Message {
	mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[2]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use MetaPatch.ProtoReflect.Descriptor instead.
func (*MetaPatch) Descriptor() ([]byte, []int) {
	return file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescGZIP(), []int{2}
}

func (x *MetaPatch) GetLabels() map[string]string {
	if x != nil {
		return x.Labels
	}
	return nil
}

func (x *MetaPatch) GetAnnotations() map[string]string {
	if x != nil {
		return x.Annotations
	}
	return nil
}

// GameServerSelector used for finding a GameServer with matching labels.
type GameServerSelector struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Labels to match.
	MatchLabels map[string]string `protobuf:"bytes,1,rep,name=match_labels,json=matchLabels,proto3" json:"match_labels,omitempty" protobuf_key:"bytes,1,opt,name=key,proto3" protobuf_val:"bytes,2,opt,name=value,proto3"`
}

func (x *GameServerSelector) Reset() {
	*x = GameServerSelector{}
	if protoimpl.UnsafeEnabled {
		mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[3]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GameServerSelector) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GameServerSelector) ProtoMessage() {}

func (x *GameServerSelector) ProtoReflect() protoreflect.Message {
	mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[3]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GameServerSelector.ProtoReflect.Descriptor instead.
func (*GameServerSelector) Descriptor() ([]byte, []int) {
	return file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescGZIP(), []int{3}
}

func (x *GameServerSelector) GetMatchLabels() map[string]string {
	if x != nil {
		return x.MatchLabels
	}
	return nil
}

// The game server port info that is allocated.
type AllocationResponse_GameServerStatusPort struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
	Port int32  `protobuf:"varint,2,opt,name=port,proto3" json:"port,omitempty"`
}

func (x *AllocationResponse_GameServerStatusPort) Reset() {
	*x = AllocationResponse_GameServerStatusPort{}
	if protoimpl.UnsafeEnabled {
		mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[4]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *AllocationResponse_GameServerStatusPort) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*AllocationResponse_GameServerStatusPort) ProtoMessage() {}

func (x *AllocationResponse_GameServerStatusPort) ProtoReflect() protoreflect.Message {
	mi := &file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[4]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use AllocationResponse_GameServerStatusPort.ProtoReflect.Descriptor instead.
func (*AllocationResponse_GameServerStatusPort) Descriptor() ([]byte, []int) {
	return file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescGZIP(), []int{1, 0}
}

func (x *AllocationResponse_GameServerStatusPort) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

func (x *AllocationResponse_GameServerStatusPort) GetPort() int32 {
	if x != nil {
		return x.Port
	}
	return 0
}

var File_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto protoreflect.FileDescriptor

var file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDesc = []byte{
	0x0a, 0x48, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2f, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2f, 0x67,
	0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2f, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e,
	0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2f, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61,
	0x2f, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x5f, 0x65, 0x6e, 0x64, 0x70,
	0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x12, 0x2e, 0x67, 0x6f, 0x6f, 0x67,
	0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2e,
	0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69,
	0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x1a, 0x1c, 0x67, 0x6f, 0x6f, 0x67,
	0x6c, 0x65, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x61, 0x6e, 0x6e, 0x6f, 0x74, 0x61, 0x74, 0x69, 0x6f,
	0x6e, 0x73, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x1a, 0x17, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65,
	0x2f, 0x61, 0x70, 0x69, 0x2f, 0x63, 0x6c, 0x69, 0x65, 0x6e, 0x74, 0x2e, 0x70, 0x72, 0x6f, 0x74,
	0x6f, 0x22, 0xbd, 0x02, 0x0a, 0x11, 0x41, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e,
	0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x14, 0x0a, 0x05, 0x72, 0x65, 0x61, 0x6c, 0x6d,
	0x18, 0x01, 0x20, 0x01, 0x28, 0x09, 0x52, 0x05, 0x72, 0x65, 0x61, 0x6c, 0x6d, 0x12, 0x43, 0x0a,
	0x1e, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6c, 0x74, 0x5f, 0x67, 0x61, 0x6d, 0x65, 0x5f, 0x73, 0x65,
	0x72, 0x76, 0x65, 0x72, 0x5f, 0x64, 0x65, 0x70, 0x6c, 0x6f, 0x79, 0x6d, 0x65, 0x6e, 0x74, 0x18,
	0x02, 0x20, 0x01, 0x28, 0x09, 0x52, 0x1b, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6c, 0x74, 0x47, 0x61,
	0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x44, 0x65, 0x70, 0x6c, 0x6f, 0x79, 0x6d, 0x65,
	0x6e, 0x74, 0x12, 0x76, 0x0a, 0x15, 0x67, 0x61, 0x6d, 0x65, 0x5f, 0x73, 0x65, 0x72, 0x76, 0x65,
	0x72, 0x5f, 0x73, 0x65, 0x6c, 0x65, 0x63, 0x74, 0x6f, 0x72, 0x73, 0x18, 0x03, 0x20, 0x03, 0x28,
	0x0b, 0x32, 0x42, 0x2e, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64,
	0x2e, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69,
	0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70,
	0x68, 0x61, 0x2e, 0x47, 0x61, 0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x53, 0x65, 0x6c,
	0x65, 0x63, 0x74, 0x6f, 0x72, 0x52, 0x13, 0x67, 0x61, 0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65,
	0x72, 0x53, 0x65, 0x6c, 0x65, 0x63, 0x74, 0x6f, 0x72, 0x73, 0x12, 0x55, 0x0a, 0x08, 0x6d, 0x65,
	0x74, 0x61, 0x64, 0x61, 0x74, 0x61, 0x18, 0x04, 0x20, 0x01, 0x28, 0x0b, 0x32, 0x39, 0x2e, 0x67,
	0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d, 0x69,
	0x6e, 0x67, 0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64,
	0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x2e, 0x4d, 0x65,
	0x74, 0x61, 0x50, 0x61, 0x74, 0x63, 0x68, 0x52, 0x08, 0x6d, 0x65, 0x74, 0x61, 0x64, 0x61, 0x74,
	0x61, 0x22, 0x86, 0x03, 0x0a, 0x12, 0x41, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e,
	0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x12, 0x28, 0x0a, 0x10, 0x67, 0x61, 0x6d, 0x65,
	0x5f, 0x73, 0x65, 0x72, 0x76, 0x65, 0x72, 0x5f, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01,
	0x28, 0x09, 0x52, 0x0e, 0x67, 0x61, 0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x4e, 0x61,
	0x6d, 0x65, 0x12, 0x6d, 0x0a, 0x05, 0x70, 0x6f, 0x72, 0x74, 0x73, 0x18, 0x02, 0x20, 0x03, 0x28,
	0x0b, 0x32, 0x57, 0x2e, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64,
	0x2e, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69,
	0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70,
	0x68, 0x61, 0x2e, 0x41, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x73,
	0x70, 0x6f, 0x6e, 0x73, 0x65, 0x2e, 0x47, 0x61, 0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72,
	0x53, 0x74, 0x61, 0x74, 0x75, 0x73, 0x50, 0x6f, 0x72, 0x74, 0x52, 0x05, 0x70, 0x6f, 0x72, 0x74,
	0x73, 0x12, 0x18, 0x0a, 0x07, 0x61, 0x64, 0x64, 0x72, 0x65, 0x73, 0x73, 0x18, 0x03, 0x20, 0x01,
	0x28, 0x09, 0x52, 0x07, 0x61, 0x64, 0x64, 0x72, 0x65, 0x73, 0x73, 0x12, 0x1b, 0x0a, 0x09, 0x6e,
	0x6f, 0x64, 0x65, 0x5f, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x04, 0x20, 0x01, 0x28, 0x09, 0x52, 0x08,
	0x6e, 0x6f, 0x64, 0x65, 0x4e, 0x61, 0x6d, 0x65, 0x12, 0x37, 0x0a, 0x18, 0x67, 0x61, 0x6d, 0x65,
	0x5f, 0x73, 0x65, 0x72, 0x76, 0x65, 0x72, 0x5f, 0x63, 0x6c, 0x75, 0x73, 0x74, 0x65, 0x72, 0x5f,
	0x6e, 0x61, 0x6d, 0x65, 0x18, 0x05, 0x20, 0x01, 0x28, 0x09, 0x52, 0x15, 0x67, 0x61, 0x6d, 0x65,
	0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x43, 0x6c, 0x75, 0x73, 0x74, 0x65, 0x72, 0x4e, 0x61, 0x6d,
	0x65, 0x12, 0x27, 0x0a, 0x0f, 0x64, 0x65, 0x70, 0x6c, 0x6f, 0x79, 0x6d, 0x65, 0x6e, 0x74, 0x5f,
	0x6e, 0x61, 0x6d, 0x65, 0x18, 0x06, 0x20, 0x01, 0x28, 0x09, 0x52, 0x0e, 0x64, 0x65, 0x70, 0x6c,
	0x6f, 0x79, 0x6d, 0x65, 0x6e, 0x74, 0x4e, 0x61, 0x6d, 0x65, 0x1a, 0x3e, 0x0a, 0x14, 0x47, 0x61,
	0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x53, 0x74, 0x61, 0x74, 0x75, 0x73, 0x50, 0x6f,
	0x72, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01, 0x28, 0x09,
	0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x12, 0x12, 0x0a, 0x04, 0x70, 0x6f, 0x72, 0x74, 0x18, 0x02,
	0x20, 0x01, 0x28, 0x05, 0x52, 0x04, 0x70, 0x6f, 0x72, 0x74, 0x22, 0xd3, 0x02, 0x0a, 0x09, 0x4d,
	0x65, 0x74, 0x61, 0x50, 0x61, 0x74, 0x63, 0x68, 0x12, 0x5d, 0x0a, 0x06, 0x6c, 0x61, 0x62, 0x65,
	0x6c, 0x73, 0x18, 0x01, 0x20, 0x03, 0x28, 0x0b, 0x32, 0x45, 0x2e, 0x67, 0x6f, 0x6f, 0x67, 0x6c,
	0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2e, 0x61,
	0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e,
	0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x2e, 0x4d, 0x65, 0x74, 0x61, 0x50, 0x61,
	0x74, 0x63, 0x68, 0x2e, 0x4c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x45, 0x6e, 0x74, 0x72, 0x79, 0x52,
	0x06, 0x6c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x12, 0x6c, 0x0a, 0x0b, 0x61, 0x6e, 0x6e, 0x6f, 0x74,
	0x61, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x18, 0x02, 0x20, 0x03, 0x28, 0x0b, 0x32, 0x4a, 0x2e, 0x67,
	0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d, 0x69,
	0x6e, 0x67, 0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64,
	0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x2e, 0x4d, 0x65,
	0x74, 0x61, 0x50, 0x61, 0x74, 0x63, 0x68, 0x2e, 0x41, 0x6e, 0x6e, 0x6f, 0x74, 0x61, 0x74, 0x69,
	0x6f, 0x6e, 0x73, 0x45, 0x6e, 0x74, 0x72, 0x79, 0x52, 0x0b, 0x61, 0x6e, 0x6e, 0x6f, 0x74, 0x61,
	0x74, 0x69, 0x6f, 0x6e, 0x73, 0x1a, 0x39, 0x0a, 0x0b, 0x4c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x45,
	0x6e, 0x74, 0x72, 0x79, 0x12, 0x10, 0x0a, 0x03, 0x6b, 0x65, 0x79, 0x18, 0x01, 0x20, 0x01, 0x28,
	0x09, 0x52, 0x03, 0x6b, 0x65, 0x79, 0x12, 0x14, 0x0a, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x18,
	0x02, 0x20, 0x01, 0x28, 0x09, 0x52, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x3a, 0x02, 0x38, 0x01,
	0x1a, 0x3e, 0x0a, 0x10, 0x41, 0x6e, 0x6e, 0x6f, 0x74, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x45,
	0x6e, 0x74, 0x72, 0x79, 0x12, 0x10, 0x0a, 0x03, 0x6b, 0x65, 0x79, 0x18, 0x01, 0x20, 0x01, 0x28,
	0x09, 0x52, 0x03, 0x6b, 0x65, 0x79, 0x12, 0x14, 0x0a, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x18,
	0x02, 0x20, 0x01, 0x28, 0x09, 0x52, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x3a, 0x02, 0x38, 0x01,
	0x22, 0xcc, 0x01, 0x0a, 0x12, 0x47, 0x61, 0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x53,
	0x65, 0x6c, 0x65, 0x63, 0x74, 0x6f, 0x72, 0x12, 0x76, 0x0a, 0x0c, 0x6d, 0x61, 0x74, 0x63, 0x68,
	0x5f, 0x6c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x18, 0x01, 0x20, 0x03, 0x28, 0x0b, 0x32, 0x53, 0x2e,
	0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d,
	0x69, 0x6e, 0x67, 0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e,
	0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x2e, 0x47,
	0x61, 0x6d, 0x65, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x53, 0x65, 0x6c, 0x65, 0x63, 0x74, 0x6f,
	0x72, 0x2e, 0x4d, 0x61, 0x74, 0x63, 0x68, 0x4c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x45, 0x6e, 0x74,
	0x72, 0x79, 0x52, 0x0b, 0x6d, 0x61, 0x74, 0x63, 0x68, 0x4c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x1a,
	0x3e, 0x0a, 0x10, 0x4d, 0x61, 0x74, 0x63, 0x68, 0x4c, 0x61, 0x62, 0x65, 0x6c, 0x73, 0x45, 0x6e,
	0x74, 0x72, 0x79, 0x12, 0x10, 0x0a, 0x03, 0x6b, 0x65, 0x79, 0x18, 0x01, 0x20, 0x01, 0x28, 0x09,
	0x52, 0x03, 0x6b, 0x65, 0x79, 0x12, 0x14, 0x0a, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x18, 0x02,
	0x20, 0x01, 0x28, 0x09, 0x52, 0x05, 0x76, 0x61, 0x6c, 0x75, 0x65, 0x3a, 0x02, 0x38, 0x01, 0x32,
	0xc1, 0x01, 0x0a, 0x19, 0x41, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x45, 0x6e,
	0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x12, 0x93, 0x01,
	0x0a, 0x08, 0x41, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x65, 0x12, 0x41, 0x2e, 0x67, 0x6f, 0x6f,
	0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67,
	0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f,
	0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x2e, 0x41, 0x6c, 0x6c, 0x6f,
	0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a, 0x42, 0x2e,
	0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d,
	0x69, 0x6e, 0x67, 0x2e, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e,
	0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x2e, 0x41,
	0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73,
	0x65, 0x22, 0x00, 0x1a, 0x0e, 0xca, 0x41, 0x0b, 0x65, 0x78, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x2e,
	0x63, 0x6f, 0x6d, 0x42, 0xb1, 0x01, 0x0a, 0x32, 0x63, 0x6f, 0x6d, 0x2e, 0x67, 0x6f, 0x6f, 0x67,
	0x6c, 0x65, 0x2e, 0x63, 0x6c, 0x6f, 0x75, 0x64, 0x2e, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2e,
	0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69,
	0x6e, 0x74, 0x2e, 0x76, 0x31, 0x61, 0x6c, 0x70, 0x68, 0x61, 0x42, 0x17, 0x41, 0x6c, 0x6c, 0x6f,
	0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x45, 0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x72,
	0x6f, 0x74, 0x6f, 0x50, 0x01, 0x5a, 0x60, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2e, 0x67, 0x6f,
	0x6c, 0x61, 0x6e, 0x67, 0x2e, 0x6f, 0x72, 0x67, 0x2f, 0x67, 0x65, 0x6e, 0x70, 0x72, 0x6f, 0x74,
	0x6f, 0x2f, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x61, 0x70, 0x69, 0x73, 0x2f, 0x63, 0x6c, 0x6f,
	0x75, 0x64, 0x2f, 0x67, 0x61, 0x6d, 0x69, 0x6e, 0x67, 0x2f, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61,
	0x74, 0x69, 0x6f, 0x6e, 0x65, 0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x2f, 0x76, 0x31, 0x61,
	0x6c, 0x70, 0x68, 0x61, 0x3b, 0x61, 0x6c, 0x6c, 0x6f, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x65,
	0x6e, 0x64, 0x70, 0x6f, 0x69, 0x6e, 0x74, 0x62, 0x06, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x33,
}

var (
	file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescOnce sync.Once
	file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescData = file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDesc
)

func file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescGZIP() []byte {
	file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescOnce.Do(func() {
		file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescData = protoimpl.X.CompressGZIP(file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescData)
	})
	return file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDescData
}

var file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes = make([]protoimpl.MessageInfo, 8)
var file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_goTypes = []interface{}{
	(*AllocationRequest)(nil),                       // 0: google.cloud.gaming.allocationendpoint.v1alpha.AllocationRequest
	(*AllocationResponse)(nil),                      // 1: google.cloud.gaming.allocationendpoint.v1alpha.AllocationResponse
	(*MetaPatch)(nil),                               // 2: google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch
	(*GameServerSelector)(nil),                      // 3: google.cloud.gaming.allocationendpoint.v1alpha.GameServerSelector
	(*AllocationResponse_GameServerStatusPort)(nil), // 4: google.cloud.gaming.allocationendpoint.v1alpha.AllocationResponse.GameServerStatusPort
	nil, // 5: google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch.LabelsEntry
	nil, // 6: google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch.AnnotationsEntry
	nil, // 7: google.cloud.gaming.allocationendpoint.v1alpha.GameServerSelector.MatchLabelsEntry
}
var file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_depIdxs = []int32{
	3, // 0: google.cloud.gaming.allocationendpoint.v1alpha.AllocationRequest.game_server_selectors:type_name -> google.cloud.gaming.allocationendpoint.v1alpha.GameServerSelector
	2, // 1: google.cloud.gaming.allocationendpoint.v1alpha.AllocationRequest.metadata:type_name -> google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch
	4, // 2: google.cloud.gaming.allocationendpoint.v1alpha.AllocationResponse.ports:type_name -> google.cloud.gaming.allocationendpoint.v1alpha.AllocationResponse.GameServerStatusPort
	5, // 3: google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch.labels:type_name -> google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch.LabelsEntry
	6, // 4: google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch.annotations:type_name -> google.cloud.gaming.allocationendpoint.v1alpha.MetaPatch.AnnotationsEntry
	7, // 5: google.cloud.gaming.allocationendpoint.v1alpha.GameServerSelector.match_labels:type_name -> google.cloud.gaming.allocationendpoint.v1alpha.GameServerSelector.MatchLabelsEntry
	0, // 6: google.cloud.gaming.allocationendpoint.v1alpha.AllocationEndpointService.Allocate:input_type -> google.cloud.gaming.allocationendpoint.v1alpha.AllocationRequest
	1, // 7: google.cloud.gaming.allocationendpoint.v1alpha.AllocationEndpointService.Allocate:output_type -> google.cloud.gaming.allocationendpoint.v1alpha.AllocationResponse
	7, // [7:8] is the sub-list for method output_type
	6, // [6:7] is the sub-list for method input_type
	6, // [6:6] is the sub-list for extension type_name
	6, // [6:6] is the sub-list for extension extendee
	0, // [0:6] is the sub-list for field type_name
}

func init() { file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_init() }
func file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_init() {
	if File_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*AllocationRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*AllocationResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[2].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*MetaPatch); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[3].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GameServerSelector); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes[4].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*AllocationResponse_GameServerStatusPort); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
	}
	type x struct{}
	out := protoimpl.TypeBuilder{
		File: protoimpl.DescBuilder{
			GoPackagePath: reflect.TypeOf(x{}).PkgPath(),
			RawDescriptor: file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   8,
			NumExtensions: 0,
			NumServices:   1,
		},
		GoTypes:           file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_goTypes,
		DependencyIndexes: file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_depIdxs,
		MessageInfos:      file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_msgTypes,
	}.Build()
	File_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto = out.File
	file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_rawDesc = nil
	file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_goTypes = nil
	file_google_cloud_gaming_allocationendpoint_v1alpha_allocation_endpoint_proto_depIdxs = nil
}

// Reference imports to suppress errors if they are not otherwise used.
var _ context.Context
var _ grpc.ClientConnInterface

// This is a compile-time assertion to ensure that this generated file
// is compatible with the grpc package it is being compiled against.
const _ = grpc.SupportPackageIsVersion6

// AllocationEndpointServiceClient is the client API for AllocationEndpointService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://godoc.org/google.golang.org/grpc#ClientConn.NewStream.
type AllocationEndpointServiceClient interface {
	// Proxy allocation service to the Game Server Clusters.
	Allocate(ctx context.Context, in *AllocationRequest, opts ...grpc.CallOption) (*AllocationResponse, error)
}

type allocationEndpointServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewAllocationEndpointServiceClient(cc grpc.ClientConnInterface) AllocationEndpointServiceClient {
	return &allocationEndpointServiceClient{cc}
}

func (c *allocationEndpointServiceClient) Allocate(ctx context.Context, in *AllocationRequest, opts ...grpc.CallOption) (*AllocationResponse, error) {
	out := new(AllocationResponse)
	err := c.cc.Invoke(ctx, "/google.cloud.gaming.allocationendpoint.v1alpha.AllocationEndpointService/Allocate", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

// AllocationEndpointServiceServer is the server API for AllocationEndpointService service.
type AllocationEndpointServiceServer interface {
	// Proxy allocation service to the Game Server Clusters.
	Allocate(context.Context, *AllocationRequest) (*AllocationResponse, error)
}

// UnimplementedAllocationEndpointServiceServer can be embedded to have forward compatible implementations.
type UnimplementedAllocationEndpointServiceServer struct {
}

func (*UnimplementedAllocationEndpointServiceServer) Allocate(context.Context, *AllocationRequest) (*AllocationResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method Allocate not implemented")
}

func RegisterAllocationEndpointServiceServer(s *grpc.Server, srv AllocationEndpointServiceServer) {
	s.RegisterService(&_AllocationEndpointService_serviceDesc, srv)
}

func _AllocationEndpointService_Allocate_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(AllocationRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(AllocationEndpointServiceServer).Allocate(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/google.cloud.gaming.allocationendpoint.v1alpha.AllocationEndpointService/Allocate",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(AllocationEndpointServiceServer).Allocate(ctx, req.(*AllocationRequest))
	}
	return interceptor(ctx, in, info, handler)
}

var _AllocationEndpointService_serviceDesc = grpc.ServiceDesc{
	ServiceName: "google.cloud.gaming.allocationendpoint.v1alpha.AllocationEndpointService",
	HandlerType: (*AllocationEndpointServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "Allocate",
			Handler:    _AllocationEndpointService_Allocate_Handler,
		},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "google/cloud/gaming/allocationendpoint/v1alpha/allocation_endpoint.proto",
}