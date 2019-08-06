add_swift_host_tool(replv2
  replv2.cpp
  REPLEnv.cpp
  LLVM_LINK_COMPONENTS
    Core
    IRReader
    OrcJIT
    Support
    nativecodegen
  SWIFT_COMPONENT tools
)
#target_link_libraries(swift-reflection-dump
#                      PRIVATE
#                        swiftReflection)
