//
// Created by jemin on 1/13/21.
//

#ifndef _RAISIM_GYM_TORCH_RAISIMGYMTORCH_ENV_BASICEIGENTYPES_HPP_
#define _RAISIM_GYM_TORCH_RAISIMGYMTORCH_ENV_BASICEIGENTYPES_HPP_

namespace raisim {

using Dtype = float;
using EigenRowMajorMat = Eigen::Matrix<Dtype, -1, -1, Eigen::RowMajor>;
using EigenVec = Eigen::Matrix<Dtype, -1, 1>;
using EigenBoolVec = Eigen::Matrix<bool, -1, 1>;

using EigenDoubleVec = Eigen::Matrix<double, -1, 1>;

#define __RSG_MAKE_STR(x) #x
#define _RSG_MAKE_STR(x) __RSG_MAKE_STR(x)
#define RSG_MAKE_STR(x) _RSG_MAKE_STR(x)

#define READ_YAML(a, b, c) RSFATAL_IF(c.IsNone(), "Node "<<RSG_MAKE_STR(c)<<" doesn't exist") b = c.template As<a>();

}

#endif //_RAISIM_GYM_TORCH_RAISIMGYMTORCH_ENV_BASICEIGENTYPES_HPP_
