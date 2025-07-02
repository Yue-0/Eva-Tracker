import os

import tensorrt as trt

__all__ = ["onnx2trt"]
__author__ = "YueLin"


def convert(onnx: str) -> str:
    assert onnx.endswith(".onnx")
    return onnx[:-4] + "trt"


def onnx2trt(onnx: str, half: bool = False, flags: int = None) -> None:
    logger = trt.Logger()
    if flags is None:
        flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    with trt.Builder(logger) as builder:
        with builder.create_network(flags=flags) as net:
            with trt.OnnxParser(net, logger) as parser:
                with open(onnx, "rb") as model:
                    if parser.parse(model.read()):
                        print("Building an engine. This would take a while...")
                        config = builder.create_builder_config()
                        config.set_memory_pool_limit(
                            trt.MemoryPoolType.WORKSPACE, 2 << 30
                        )
                        if half:
                            config.flags |= 1 << int(trt.BuilderFlag.FP16)
                        engine = builder.build_serialized_network(net, config)
                    else:
                        for error in range(parser.num_errors):
                            print(parser.get_error(error))
                        raise SystemExit("Failed to build the TensorRT engine!")
    onnx = convert(onnx)
    with open(onnx, "wb") as f:
        f.write(engine)
    print("Serialized the TensorRT engine to file:", onnx)


if __name__ == "__main__":
    path = os.path.join(os.path.dirname(__file__), "src", "tracker", "models")
    names = os.listdir(path)
    for name in names:
        if name.endswith(".onnx") and convert(name) not in names:
            print("Try converting {} to TensorRT engine.".format(name))
            onnx2trt(os.path.join(path, name))
