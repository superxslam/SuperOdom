"""Utility functions supporting the other modules"""

import torch
import numpy as np

def norm_std(x):
  x = x - x.mean()
  s = x.std()
  if s == 0:
    return torch.ones_like(x)*0.5
  else:
    return x / s

def norm_01(x):
  x = x - x.min()
  mx = x.max()
  if mx == 0:
    return torch.ones_like(x)*0.5
  else:
    return x / mx

def norm_img_01(x):
  B,C,H,W = x.shape
  x = x - torch.min(x.reshape(B,C, H*W), dim=-1).values.reshape(B,C,1,1)
  x = x / torch.max(x.reshape(B,C, H*W), dim=-1).values.reshape(B,C,1,1)
  return x

def compute_cos_sim(vec1: torch.FloatTensor,
                    vec2: torch.FloatTensor,
                    softmax: bool = False) -> torch.FloatTensor:
  """Compute cosine similarity between two batches of D dim vectors.
  
  Args:
    vec1: NxC float tensor representing batch of vectors
    vec2: MxC float tensor representing batch of vectors
    softmax: If False, cosine similarity is returned. If True, softmaxed
      probability is returned.
  Returns:
    result: MxN float tensor representing similarity/prob. where result[0,1]
      represents the similarity of vec1[0] with vec2[1]
  """
  N, C1 = vec1.shape
  M, C2 = vec2.shape
  if C1 != C2:
    raise ValueError(f"vec1 feature dimension '{C1}' does not match vec2"
                     f"feature dimension '{C2}'")
  C = C1

  vec1 = vec1 / vec1.norm(dim = -1, keepdim = True)
  vec1 = vec1.reshape(1, N, 1, C)

  vec2 = vec2 / vec2.norm(dim=-1, keepdim = True)
  vec2 = vec2.reshape(M, 1, C, 1)

  sim = (vec1 @ vec2).reshape(M, N)
  if softmax:
    return torch.softmax(100 * sim, dim = -1)
  else:
    return sim

def quaternion_to_rotation_matrix(q):
    """Convert quaternion to rotation matrix without external dependencies.
    
    Args:
        q: Quaternion in form [w, x, y, z]
        
    Returns:
        3x3 rotation matrix as numpy array
    """
    w, x, y, z = q
    
    # Calculate rotation matrix elements
    xx, xy, xz = x*x, x*y, x*z
    yy, yz, zz = y*y, y*z, z*z
    wx, wy, wz = w*x, w*y, w*z
    
    # Build the rotation matrix
    R = np.zeros((3, 3), dtype=np.float32)
    R[0, 0] = 1 - 2*(yy + zz)
    R[0, 1] = 2*(xy - wz)
    R[0, 2] = 2*(xz + wy)
    R[1, 0] = 2*(xy + wz)
    R[1, 1] = 1 - 2*(xx + zz)
    R[1, 2] = 2*(yz - wx)
    R[2, 0] = 2*(xz - wy)
    R[2, 1] = 2*(yz + wx)
    R[2, 2] = 1 - 2*(xx + yy)
    
    return R