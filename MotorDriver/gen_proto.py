import os

Import("env")

# env.Execute("echo \"asdf\"")
print("Generating embedded protobuf files")
embedded_proto_path = "lib/EmbeddedProto"
lib_base_path = "lib/IRISMotorDriver"
env.Execute(f"protoc --plugin=protoc-gen-eams={embedded_proto_path}/protoc-gen-eams" + " "
            + f"-I{lib_base_path}/proto" + " "
            + f"--eams_out={lib_base_path}/include/generated" + " "
            + f"{lib_base_path}/proto/commands.proto")

# env.BuildSources(
#     os.path.join("$BUILD_DIR", "external", "build"),
#     os.path.join("$PROJECT_DIR", "external", "sources")
# )