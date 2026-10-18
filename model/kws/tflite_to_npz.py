# Dequantise MicroNet-M's int8 weights into float32 arrays, in layer order (one-off;
# run with the litert venv). Output: w{i}, b{i} per conv layer, OHWI / depthwise 1HWC.
import sys, numpy as np, tflite
from ai_edge_litert.interpreter import Interpreter
path, out_path = sys.argv[1], sys.argv[2]
it = Interpreter(path); it.allocate_tensors()
det = {d['index']: d for d in it.get_tensor_details()}
def deq(idx):
    d = det[idx]; q = d['quantization_parameters']; v = it.get_tensor(idx).astype(np.float32)
    sc, zp, ax = q['scales'], q['zero_points'], q['quantized_dimension']
    if sc.size == 1: return (v - zp[0]) * sc[0]
    shp = [1] * v.ndim; shp[ax] = -1
    return (v - zp.reshape(shp)) * sc.reshape(shp)
m = tflite.Model.GetRootAsModel(open(path, 'rb').read(), 0); g = m.Subgraphs(0)
out, n = {}, 0
for i in range(g.OperatorsLength()):
    op = g.Operators(i)
    code = m.OperatorCodes(op.OpcodeIndex()).BuiltinCode() or m.OperatorCodes(op.OpcodeIndex()).DeprecatedBuiltinCode()
    if code in (tflite.BuiltinOperator.CONV_2D, tflite.BuiltinOperator.DEPTHWISE_CONV_2D):
        out[f'w{n}'] = deq(op.Inputs(1)); out[f'b{n}'] = deq(op.Inputs(2))
        print(n, 'dw' if code == tflite.BuiltinOperator.DEPTHWISE_CONV_2D else 'conv', out[f'w{n}'].shape); n += 1
inp = det[g.Inputs(0)]['quantization_parameters']; outq = det[g.Outputs(0)]['quantization_parameters']
out['in_q'] = np.array([inp['scales'][0], inp['zero_points'][0]], np.float32)
out['out_q'] = np.array([outq['scales'][0], outq['zero_points'][0]], np.float32)
np.savez(out_path, **out)
