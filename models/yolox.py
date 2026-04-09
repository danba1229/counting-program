import numpy as np
import cv2

try:
    import onnxruntime as ort
except Exception:
    ort = None


def _choose_runtime_providers(providers=None):
    if providers is not None:
        return providers
    if ort is None:
        return None
    available = list(ort.get_available_providers())
    selected = []
    for provider in ["DmlExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]:
        if provider in available:
            selected.append(provider)
    return selected or ["CPUExecutionProvider"]

class YoloX:
    def __init__(
        self,
        modelPath,
        confThreshold=0.35,
        nmsThreshold=0.5,
        objThreshold=0.5,
        backendId=0,
        targetId=0,
        providers=None,
    ):
        self.num_classes = 80
        self.net = None
        self.session = None
        self.input_name = None
        self.input_size = (640, 640)
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)
        self.strides = [8, 16, 32]
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold
        self.backendId = backendId
        self.targetId = targetId
        self.runtime_label = "OpenCV DNN"
        self.device_label = "CPU"

        if ort is not None:
            try:
                ort.preload_dlls()
            except Exception:
                pass
            runtime_providers = _choose_runtime_providers(providers)
            try:
                self.session = ort.InferenceSession(modelPath, providers=runtime_providers)
                self.input_name = self.session.get_inputs()[0].name
                active_providers = self.session.get_providers()
                self.runtime_label = "ONNX Runtime"
                if active_providers and active_providers[0] == "DmlExecutionProvider":
                    self.device_label = "GPU(DirectML)"
                elif active_providers and active_providers[0] == "CUDAExecutionProvider":
                    self.device_label = "GPU(CUDA)"
                else:
                    self.device_label = "CPU"
            except Exception:
                self.session = None
                self.input_name = None

        if self.session is None:
            self.net = cv2.dnn.readNet(modelPath)
            self.net.setPreferableBackend(self.backendId)
            self.net.setPreferableTarget(self.targetId)
            if self.targetId in (getattr(cv2.dnn, "DNN_TARGET_CUDA", -1), getattr(cv2.dnn, "DNN_TARGET_CUDA_FP16", -1)):
                self.device_label = "GPU(CUDA)"

        self.generateAnchors()

    @property
    def name(self):
        return self.__class__.__name__

    def setBackendAndTarget(self, backendId, targetId):
        self.backendId = backendId
        self.targetId = targetId
        if self.net is not None:
            self.net.setPreferableBackend(self.backendId)
            self.net.setPreferableTarget(self.targetId)

    def preprocess(self, img):
        blob = np.transpose(img, (2, 0, 1))
        return np.ascontiguousarray(blob[np.newaxis, :, :, :], dtype=np.float32)

    def infer(self, srcimg):
        input_blob = self.preprocess(srcimg)

        if self.session is not None:
            outs = self.session.run(None, {self.input_name: input_blob})
            predictions = self.postprocess(outs[0])
            return predictions

        self.net.setInput(input_blob)
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        predictions = self.postprocess(outs[0])
        return predictions

    def postprocess(self, outputs):
        dets = outputs[0]

        dets[:, :2] = (dets[:, :2] + self.grids) * self.expanded_strides
        dets[:, 2:4] = np.exp(dets[:, 2:4]) * self.expanded_strides

        # get boxes
        boxes = dets[:, :4]
        boxes_xywh = np.ones_like(boxes)
        boxes_xywh[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.
        boxes_xywh[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.
        boxes_xywh[:, 2] = boxes[:, 2]
        boxes_xywh[:, 3] = boxes[:, 3]

        # get scores and class indices
        scores = dets[:, 4:5] * dets[:, 5:]
        max_scores = np.amax(scores, axis=1)
        max_scores_idx = np.argmax(scores, axis=1)

        keep = cv2.dnn.NMSBoxesBatched(boxes_xywh.tolist(), max_scores.tolist(), max_scores_idx.tolist(), self.confThreshold, self.nmsThreshold)

        candidates = np.concatenate([boxes_xywh, max_scores[:, None], max_scores_idx[:, None]], axis=1)
        if len(keep) == 0:
            return np.array([])
        return candidates[keep]

    def generateAnchors(self):
        self.grids = []
        self.expanded_strides = []
        hsizes = [self.input_size[0] // stride for stride in self.strides]
        wsizes = [self.input_size[1] // stride for stride in self.strides]

        for hsize, wsize, stride in zip(hsizes, wsizes, self.strides):
            xv, yv = np.meshgrid(np.arange(hsize), np.arange(wsize))
            grid = np.stack((xv, yv), 2).reshape(1, -1, 2)
            self.grids.append(grid)
            shape = grid.shape[:2]
            self.expanded_strides.append(np.full((*shape, 1), stride))

        self.grids = np.concatenate(self.grids, 1)
        self.expanded_strides = np.concatenate(self.expanded_strides, 1)
