import os

Import("env")

print("Removing old embedded protobuf files")
env.Execute("rm lib/IRISMotorDriver/include/generated/*.h")
print("Generating embedded protobuf files")
embedded_proto_path = "lib/EmbeddedProto"
lib_base_path = "lib/IRISMotorDriver"
env.Execute(f"protoc --plugin=protoc-gen-eams={embedded_proto_path}/protoc-gen-eams" + " "
            + f"-I{lib_base_path}/proto" + " "
            + f"-I{embedded_proto_path}/generator" + " "
            + f"--eams_out={lib_base_path}/include/generated" + " "
            + f"{lib_base_path}/proto/*.proto")