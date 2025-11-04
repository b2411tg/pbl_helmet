from rknnlite.api import RKNNLite

RKNN_MODEL = 'yolo/best_rk3588_320.rknn'

rknn = RKNNLite()

# 1) .rknn をロード
ret = rknn.load_rknn(RKNN_MODEL)
assert ret == 0, f"load_rknn failed: {ret}"

# 2) ランタイム初期化
ret = rknn.init_runtime(target=None)#'rk3588')
assert ret == 0, f"init_runtime failed: {ret}"

print("NPU runtime OK ✅")

rknn.release()
