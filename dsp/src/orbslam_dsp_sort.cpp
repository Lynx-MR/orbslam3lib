/**
 * @file orbslam_dsp_sort.cpp
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Optimized bitonic sort algorithm using HVX instructions (Legacy)
**/


#include "orbslam_dsp_sort.h"

#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"


#include "AEEStdErr.h"

#include "orbslam3.h"

#include "hvx_internal.h"

constexpr union{
    HVX_VectorPred vectMask;
    uint16_t values[VLEN/sizeof(uint16_t)];
} bitonic_2_mask_switch_1 = {
    .values = {
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,

        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,

        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,

        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
        0xFFFF,0,0,0,
    },
},
bitonic_2_mask_switch_2 = {
    .values = {
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,

        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,

        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,

        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
        0,0,0xFFFF,0,
    },
},

bitonic_4_mask_switch_1_1 = {
    .values = {
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0,0,0,0,
    },
},
bitonic_4_mask_switch_1_2 = {
    .values = {
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0xFFFF,0xFFFF,0,0,
    },
},

bitonic_4_mask_switch_2_1 = {
    .values = {
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0,0,0,0,
    },
},
bitonic_4_mask_switch_2_2 = {
    .values = {
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0xFFFF,0,0xFFFF,0,
    },
},
bitonic_8_mask_switch_1_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_8_mask_switch_1_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
    },
},
bitonic_8_mask_switch_2_1 = {
    .values = {
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_8_mask_switch_2_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
    },
},
bitonic_8_mask_switch_3_1 = {
    .values = {
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_8_mask_switch_3_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
    },
},

bitonic_16_mask_switch_1_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_16_mask_switch_1_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,
    },
},
bitonic_16_mask_switch_2_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_16_mask_switch_2_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
    },
},
bitonic_16_mask_switch_3_1 = {
    .values = {
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_16_mask_switch_3_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
    },
},
bitonic_16_mask_switch_4_1 = {
    .values = {
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_16_mask_switch_4_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
    },
},



bitonic_32_mask_switch_1_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_32_mask_switch_1_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},

bitonic_32_mask_switch_2_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_32_mask_switch_2_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0,0,0,0,
    },
},
bitonic_32_mask_switch_3_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_32_mask_switch_3_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0,0,0,0,
    },
},
bitonic_32_mask_switch_4_1 = {
    .values = {
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_32_mask_switch_4_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,0xFFFF,0xFFFF,0,0,
    },
},
bitonic_32_mask_switch_5_1 = {
    .values = {
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_32_mask_switch_5_2 = {
    .values = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0,
    },
},
bitonic_64_mask_switch_1 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_64_mask_switch_2 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    },
},
bitonic_64_mask_switch_3 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,0,0,0,0,
    },
},
bitonic_64_mask_switch_4 = {
    .values = {
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
        0xFFFF,0xFFFF,0xFFFF,0xFFFF,
        0,0,0,0,
    },
},
bitonic_64_mask_switch_5 = {
    .values = {
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
        0xFFFF,0xFFFF,0,0,
    },
},
bitonic_64_mask_switch_6 = {
    .values = {
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
        0xFFFF,0,0xFFFF,0,
    },
},
null_mask = {
    .values = {
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
    },
};

void bitonic_sort_uint16_increasing_step(HVX_Vector& keys, HVX_Vector& values, HVX_VectorPred mask1, HVX_VectorPred mask2, const int d){
    HVX_Vector rotated_right_1 = Q6_V_vror_VR(values, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1 = Q6_V_vror_VR(values, VLEN - d*sizeof(uint16_t));

    HVX_Vector rotated_right_1_keys = Q6_V_vror_VR(keys, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1_keys = Q6_V_vror_VR(keys, VLEN - d*sizeof(uint16_t));

    HVX_VectorPred greater_2 = Q6_Q_vcmp_gt_VuhVuh(rotated_right_1, values);

    HVX_VectorPred switch_2_mask_left = Q6_V_vor_VV(Q6_V_vand_VV(mask1, Q6_V_vnot_V(greater_2)),
    Q6_V_vand_VV(mask2, greater_2));
    values = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1_keys, keys);

    HVX_VectorPred switch_2_mask_right = Q6_V_vror_VR(switch_2_mask_left, VLEN - d*sizeof(uint16_t));
    values = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1_keys, keys);
}

void bitonic_sort_uint16_increasing_final_step(HVX_Vector& keys, HVX_Vector& values, HVX_VectorPred mask, const int d){
    HVX_Vector rotated_right_1 = Q6_V_vror_VR(values, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1 = Q6_V_vror_VR(values, VLEN - d*sizeof(uint16_t));

    HVX_Vector rotated_right_1_keys = Q6_V_vror_VR(keys, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1_keys = Q6_V_vror_VR(keys, VLEN - d*sizeof(uint16_t));

    HVX_VectorPred greater_2 = Q6_Q_vcmp_gt_VuhVuh(rotated_right_1, values);

    HVX_VectorPred switch_2_mask_left = Q6_V_vand_VV(mask, Q6_V_vnot_V(greater_2));
    values = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1_keys, keys);

    HVX_VectorPred switch_2_mask_right = Q6_V_vror_VR(switch_2_mask_left, VLEN - d*sizeof(uint16_t));
    values = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1_keys, keys);
}

void bitonic_sort_uint16_decreasing_step(HVX_Vector& keys, HVX_Vector& values, HVX_VectorPred mask1, HVX_VectorPred mask2, const int d){
    HVX_Vector rotated_right_1 = Q6_V_vror_VR(values, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1 = Q6_V_vror_VR(values, VLEN - d*sizeof(uint16_t));

    HVX_Vector rotated_right_1_keys = Q6_V_vror_VR(keys, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1_keys = Q6_V_vror_VR(keys, VLEN - d*sizeof(uint16_t));

    HVX_VectorPred greater_2 = Q6_Q_vcmp_gt_VuhVuh(values, rotated_right_1);

    HVX_VectorPred switch_2_mask_left = Q6_V_vor_VV(Q6_V_vand_VV(mask1, Q6_V_vnot_V(greater_2)),
    Q6_V_vand_VV(mask2, greater_2));
    values = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1_keys, keys);

    HVX_VectorPred switch_2_mask_right = Q6_V_vror_VR(switch_2_mask_left, VLEN - d*sizeof(uint16_t));
    values = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1_keys, keys);
}

void bitonic_sort_uint16_decreasing_final_step(HVX_Vector& keys, HVX_Vector& values, HVX_VectorPred mask, const int d){
    HVX_Vector rotated_right_1 = Q6_V_vror_VR(values, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1 = Q6_V_vror_VR(values, VLEN - d*sizeof(uint16_t));

    HVX_Vector rotated_right_1_keys = Q6_V_vror_VR(keys, d*sizeof(uint16_t));
    HVX_Vector rotated_left_1_keys = Q6_V_vror_VR(keys, VLEN - d*sizeof(uint16_t));

    HVX_VectorPred greater_2 = Q6_Q_vcmp_gt_VuhVuh(values, rotated_right_1);

    HVX_VectorPred switch_2_mask_left = Q6_V_vand_VV(mask, Q6_V_vnot_V(greater_2));
    values = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_left, rotated_right_1_keys, keys);

    HVX_VectorPred switch_2_mask_right = Q6_V_vror_VR(switch_2_mask_left, VLEN - d*sizeof(uint16_t));
    values = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1, values);
    keys = Q6_V_vmux_QVV(switch_2_mask_right, rotated_left_1_keys, keys);
}

void biotonic_sort_uint16_increasing_reorder_vector(HVX_Vector& keys, HVX_Vector& values){
    bitonic_sort_uint16_increasing_final_step(keys, values, bitonic_64_mask_switch_1.vectMask, 32);
    bitonic_sort_uint16_increasing_final_step(keys, values, bitonic_64_mask_switch_2.vectMask, 16);
    bitonic_sort_uint16_increasing_final_step(keys, values, bitonic_64_mask_switch_3.vectMask, 8);
    bitonic_sort_uint16_increasing_final_step(keys, values, bitonic_64_mask_switch_4.vectMask, 4);
    bitonic_sort_uint16_increasing_final_step(keys, values, bitonic_64_mask_switch_5.vectMask, 2);
    bitonic_sort_uint16_increasing_final_step(keys, values, bitonic_64_mask_switch_6.vectMask, 1);
}

void biotonic_sort_uint16_decreasing_reorder_vector(HVX_Vector& keys, HVX_Vector& values){
    bitonic_sort_uint16_decreasing_final_step(keys, values, bitonic_64_mask_switch_1.vectMask, 32);
    bitonic_sort_uint16_decreasing_final_step(keys, values, bitonic_64_mask_switch_2.vectMask, 16);
    bitonic_sort_uint16_decreasing_final_step(keys, values, bitonic_64_mask_switch_3.vectMask, 8);
    bitonic_sort_uint16_decreasing_final_step(keys, values, bitonic_64_mask_switch_4.vectMask, 4);
    bitonic_sort_uint16_decreasing_final_step(keys, values, bitonic_64_mask_switch_5.vectMask, 2);
    bitonic_sort_uint16_decreasing_final_step(keys, values, bitonic_64_mask_switch_6.vectMask, 1);
}

//LE VECTEUR EST increasing
void bitonic_sort_uint16_increasing_1v(HVX_Vector& keys, HVX_Vector& values){
    //Step 1 - Order 2 by 2
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_2_mask_switch_1.vectMask, bitonic_2_mask_switch_2.vectMask, 1);

    //Step 2 - Order 4 by 4
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_4_mask_switch_1_1.vectMask, bitonic_4_mask_switch_1_2.vectMask, 2);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_4_mask_switch_2_1.vectMask, bitonic_4_mask_switch_2_2.vectMask, 1);

    //Step 3 - Order 8 by 8
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_8_mask_switch_1_1.vectMask, bitonic_8_mask_switch_1_2.vectMask, 4);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_8_mask_switch_2_1.vectMask, bitonic_8_mask_switch_2_2.vectMask, 2);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_8_mask_switch_3_1.vectMask, bitonic_8_mask_switch_3_2.vectMask, 1);

    //Step 4 - Order 16 by 16
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_16_mask_switch_1_1.vectMask, bitonic_16_mask_switch_1_2.vectMask, 8);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_16_mask_switch_2_1.vectMask, bitonic_16_mask_switch_2_2.vectMask, 4);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_16_mask_switch_3_1.vectMask, bitonic_16_mask_switch_3_2.vectMask, 2);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_16_mask_switch_4_1.vectMask, bitonic_16_mask_switch_4_2.vectMask, 1);

    //Step 5 - Order 32 by 32
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_32_mask_switch_1_1.vectMask, bitonic_32_mask_switch_1_2.vectMask, 16);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_32_mask_switch_2_1.vectMask, bitonic_32_mask_switch_2_2.vectMask, 8);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_32_mask_switch_3_1.vectMask, bitonic_32_mask_switch_3_2.vectMask, 4);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_32_mask_switch_4_1.vectMask, bitonic_32_mask_switch_4_2.vectMask, 2);
    bitonic_sort_uint16_increasing_step(keys, values, bitonic_32_mask_switch_5_1.vectMask, bitonic_32_mask_switch_5_2.vectMask, 1);

    //Step 6 - Get a increasing vector - We need that for the next steps
    biotonic_sort_uint16_increasing_reorder_vector(keys, values);
}

//LE VECTEUR EST decreasing
void bitonic_sort_uint16_decreasing_1v(HVX_Vector& keys, HVX_Vector& values){
    //Step 1 - Order 2 by 2
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_2_mask_switch_1.vectMask, bitonic_2_mask_switch_2.vectMask, 1);

    //Step 2 - Order 4 by 4
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_4_mask_switch_1_1.vectMask, bitonic_4_mask_switch_1_2.vectMask, 2);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_4_mask_switch_2_1.vectMask, bitonic_4_mask_switch_2_2.vectMask, 1);

    //Step 3 - Order 8 by 8
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_8_mask_switch_1_1.vectMask, bitonic_8_mask_switch_1_2.vectMask, 4);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_8_mask_switch_2_1.vectMask, bitonic_8_mask_switch_2_2.vectMask, 2);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_8_mask_switch_3_1.vectMask, bitonic_8_mask_switch_3_2.vectMask, 1);

    //Step 4 - Order 16 by 16
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_16_mask_switch_1_1.vectMask, bitonic_16_mask_switch_1_2.vectMask, 8);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_16_mask_switch_2_1.vectMask, bitonic_16_mask_switch_2_2.vectMask, 4);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_16_mask_switch_3_1.vectMask, bitonic_16_mask_switch_3_2.vectMask, 2);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_16_mask_switch_4_1.vectMask, bitonic_16_mask_switch_4_2.vectMask, 1);

    //Step 5 - Order 32 by 32
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_32_mask_switch_1_1.vectMask, bitonic_32_mask_switch_1_2.vectMask, 16);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_32_mask_switch_2_1.vectMask, bitonic_32_mask_switch_2_2.vectMask, 8);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_32_mask_switch_3_1.vectMask, bitonic_32_mask_switch_3_2.vectMask, 4);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_32_mask_switch_4_1.vectMask, bitonic_32_mask_switch_4_2.vectMask, 2);
    bitonic_sort_uint16_decreasing_step(keys, values, bitonic_32_mask_switch_5_1.vectMask, bitonic_32_mask_switch_5_2.vectMask, 1);

    //Step 6 - Get a increasing vector - We need that for the next steps
    biotonic_sort_uint16_decreasing_reorder_vector(keys, values);
}

void bitonic_sort_uint16_increasing_compare_vector(HVX_Vector& keys_1, HVX_Vector& values_1, HVX_Vector& keys_2, HVX_Vector& values_2){
    HVX_VectorPred greater_2 = Q6_Q_vcmp_gt_VuhVuh(values_1, values_2);
    HVX_Vector tempValues1 = Q6_V_vmux_QVV(greater_2, values_2, values_1);
    values_2 = Q6_V_vmux_QVV(greater_2, values_1, values_2);
    values_1 = tempValues1;
    HVX_Vector tempKeys1 = Q6_V_vmux_QVV(greater_2, keys_2, keys_1);
    keys_2 = Q6_V_vmux_QVV(greater_2, keys_1, keys_2);
    keys_1 = tempKeys1;
}

void bitonic_sort_uint16_decreasing_compare_vector(HVX_Vector& keys_1, HVX_Vector& values_1, HVX_Vector& keys_2, HVX_Vector& values_2){
    HVX_VectorPred greater_2 = Q6_Q_vcmp_gt_VuhVuh(values_2, values_1);
    HVX_Vector tempValues1 = Q6_V_vmux_QVV(greater_2, values_2, values_1);
    values_2 = Q6_V_vmux_QVV(greater_2, values_1, values_2);
    values_1 = tempValues1;
    HVX_Vector tempKeys1 = Q6_V_vmux_QVV(greater_2, keys_2, keys_1);
    keys_2 = Q6_V_vmux_QVV(greater_2, keys_1, keys_2);
    keys_1 = tempKeys1;
}

void bitonic_sort_uint16_increasing_2v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_2 = keys + 1;
    HVX_Vector* values_2 = values + 1;

    bitonic_sort_uint16_increasing_1v(*keys, *values);
    bitonic_sort_uint16_decreasing_1v(*keys_2, *values_2);

    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);

    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
}

void bitonic_sort_uint16_decreasing_2v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;

    bitonic_sort_uint16_decreasing_1v(*keys, *values);
    bitonic_sort_uint16_increasing_1v(*keys_1, *values_1);

    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);

    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
}

void bitonic_sort_uint16_increasing_3v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;

    bitonic_sort_uint16_increasing_2v(keys, values);
    bitonic_sort_uint16_decreasing_1v(*keys_2, *values_2);

    //STEP 1 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);

    //STEP 2 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
}

void bitonic_sort_uint16_decreasing_3v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;

    bitonic_sort_uint16_decreasing_2v(keys, values);
    bitonic_sort_uint16_increasing_1v(*keys_2, *values_2);

    //STEP 1 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);

    //STEP 2 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
}

void bitonic_sort_uint16_increasing_4v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;

    bitonic_sort_uint16_increasing_2v(keys, values);
    bitonic_sort_uint16_decreasing_2v(keys_2, values_2);

    //STEP 1 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);

    //STEP 2 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
}

void bitonic_sort_uint16_decreasing_4v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;

    bitonic_sort_uint16_decreasing_2v(keys, values);
    bitonic_sort_uint16_increasing_2v(keys_2, values_2);

    //STEP 1 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);

    //STEP 2 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
}

void bitonic_sort_uint16_increasing_5v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;


    bitonic_sort_uint16_increasing_4v(keys, values);
    bitonic_sort_uint16_decreasing_1v(*keys_4, *values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);

    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);

    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
}

void bitonic_sort_uint16_decreasing_5v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;


    bitonic_sort_uint16_decreasing_4v(keys, values);
    bitonic_sort_uint16_increasing_1v(*keys_4, *values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);

    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);

    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
}


void bitonic_sort_uint16_increasing_6v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;

    bitonic_sort_uint16_increasing_4v(keys, values);
    bitonic_sort_uint16_decreasing_2v(keys_4, values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);

}

void bitonic_sort_uint16_decreasing_6v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;

    bitonic_sort_uint16_decreasing_4v(keys, values);
    bitonic_sort_uint16_increasing_2v(keys_4, values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);


    //STEP 2 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);

    //STEP 3 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);

}


void bitonic_sort_uint16_increasing_7v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;

    bitonic_sort_uint16_increasing_4v(keys, values);
    bitonic_sort_uint16_decreasing_3v(keys_4, values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);

    //STEP 2 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);

    //STEP 3 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
}


void bitonic_sort_uint16_decreasing_7v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;

    bitonic_sort_uint16_decreasing_4v(keys, values);
    bitonic_sort_uint16_increasing_3v(keys_4, values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);

    //STEP 2 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);

    //STEP 3 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
}

void bitonic_sort_uint16_increasing_8v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;


    bitonic_sort_uint16_increasing_4v(keys, values);
    bitonic_sort_uint16_decreasing_4v(keys_4, values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);

    //STEP 2 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);

    //STEP 3 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
}



void bitonic_sort_uint16_decreasing_8v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;


    bitonic_sort_uint16_decreasing_4v(keys, values);
    bitonic_sort_uint16_increasing_4v(keys_4, values_4);

    //STEP 1 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);

    //STEP 2 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);

    //STEP 3 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
}

void bitonic_sort_uint16_increasing_9v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_1v(*keys_8, *values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);


    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);

}

void bitonic_sort_uint16_decreasing_9v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_1v(*keys_8, *values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);


    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);

}

void bitonic_sort_uint16_increasing_10v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_2v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);

    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);


    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);

}

void bitonic_sort_uint16_decreasing_10v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_2v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);

    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);


    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);

}

void bitonic_sort_uint16_increasing_11v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_3v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);

    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);

    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);


    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);

    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);

    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_10, *values_10);


}


void bitonic_sort_uint16_decreasing_11v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_3v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);

    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);

    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);


    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);

    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);

    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_10, *values_10);


}

//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_increasing_12v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_4v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_11, *values_11);
}

//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_decreasing_12v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_4v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_11, *values_11);
}


//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_increasing_13v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_5v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_12, *values_12);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_11, *values_11, *keys_12, *values_12);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_12, *values_12);
}


//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_decreasing_13v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_5v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_12, *values_12);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_11, *values_11, *keys_12, *values_12);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_12, *values_12);
}

//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_increasing_14v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;
    HVX_Vector* keys_13 = keys + 13;
    HVX_Vector* values_13 = values + 13;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_6v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_13, *values_13);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_13, *values_13);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_11, *values_11, *keys_13, *values_13);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_11, *values_11, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_12, *values_12, *keys_13, *values_13);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_12, *values_12);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_13, *values_13);
}


//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_decreasing_14v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;
    HVX_Vector* keys_13 = keys + 13;
    HVX_Vector* values_13 = values + 13;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_6v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_13, *values_13);



    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_13, *values_13);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_11, *values_11, *keys_13, *values_13);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_11, *values_11, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_12, *values_12, *keys_13, *values_13);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_12, *values_12);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_13, *values_13);
}



//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_increasing_15v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;
    HVX_Vector* keys_13 = keys + 13;
    HVX_Vector* values_13 = values + 13;
    HVX_Vector* keys_14 = keys + 14;
    HVX_Vector* values_14 = values + 14;


    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_7v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_14, *values_14);


    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_14, *values_14);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_11, *values_11, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_12, *values_12, *keys_14, *values_14);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_11, *values_11, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_12, *values_12, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_13, *values_13, *keys_14, *values_14);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_12, *values_12);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_13, *values_13);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_14, *values_14);
}


//NOT FULLY OPTIMIZED
void bitonic_sort_uint16_decreasing_15v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;
    HVX_Vector* keys_13 = keys + 13;
    HVX_Vector* values_13 = values + 13;
    HVX_Vector* keys_14 = keys + 14;
    HVX_Vector* values_14 = values + 14;


    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_7v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_14, *values_14);



    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_14, *values_14);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_11, *values_11, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_12, *values_12, *keys_14, *values_14);


    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_11, *values_11, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_12, *values_12, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_13, *values_13, *keys_14, *values_14);



    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_12, *values_12);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_13, *values_13);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_14, *values_14);
}

void bitonic_sort_uint16_increasing_16v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;
    HVX_Vector* keys_13 = keys + 13;
    HVX_Vector* values_13 = values + 13;
    HVX_Vector* keys_14 = keys + 14;
    HVX_Vector* values_14 = values + 14;
    HVX_Vector* keys_15 = keys + 15;
    HVX_Vector* values_15 = values + 15;

    bitonic_sort_uint16_increasing_8v(keys, values);
    bitonic_sort_uint16_decreasing_8v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_14, *values_14);
    bitonic_sort_uint16_increasing_compare_vector(*keys_7, *values_7, *keys_15, *values_15);

    //STEP 2 - Delta = 4
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_14, *values_14);
    bitonic_sort_uint16_increasing_compare_vector(*keys_11, *values_11, *keys_15, *values_15);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_increasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_increasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_increasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_12, *values_12, *keys_14, *values_14);
    bitonic_sort_uint16_increasing_compare_vector(*keys_13, *values_13, *keys_15, *values_15);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_increasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_increasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_increasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_increasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_increasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_increasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_increasing_compare_vector(*keys_12, *values_12, *keys_13, *values_13);
    bitonic_sort_uint16_increasing_compare_vector(*keys_14, *values_14, *keys_15, *values_15);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_increasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_12, *values_12);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_13, *values_13);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_14, *values_14);
    biotonic_sort_uint16_increasing_reorder_vector(*keys_15, *values_15);
}




void bitonic_sort_uint16_decreasing_16v(HVX_Vector* keys, HVX_Vector* values){
    HVX_Vector* keys_1 = keys + 1;
    HVX_Vector* values_1 = values + 1;
    HVX_Vector* keys_2 = keys + 2;
    HVX_Vector* values_2 = values + 2;
    HVX_Vector* keys_3 = keys + 3;
    HVX_Vector* values_3 = values + 3;
    HVX_Vector* keys_4 = keys + 4;
    HVX_Vector* values_4 = values + 4;
    HVX_Vector* keys_5 = keys + 5;
    HVX_Vector* values_5 = values + 5;
    HVX_Vector* keys_6 = keys + 6;
    HVX_Vector* values_6 = values + 6;
    HVX_Vector* keys_7 = keys + 7;
    HVX_Vector* values_7 = values + 7;
    HVX_Vector* keys_8 = keys + 8;
    HVX_Vector* values_8 = values + 8;
    HVX_Vector* keys_9 = keys + 9;
    HVX_Vector* values_9 = values + 9;
    HVX_Vector* keys_10 = keys + 10;
    HVX_Vector* values_10 = values + 10;
    HVX_Vector* keys_11 = keys + 11;
    HVX_Vector* values_11 = values + 11;
    HVX_Vector* keys_12 = keys + 12;
    HVX_Vector* values_12 = values + 12;
    HVX_Vector* keys_13 = keys + 13;
    HVX_Vector* values_13 = values + 13;
    HVX_Vector* keys_14 = keys + 14;
    HVX_Vector* values_14 = values + 14;
    HVX_Vector* keys_15 = keys + 15;
    HVX_Vector* values_15 = values + 15;

    bitonic_sort_uint16_decreasing_8v(keys, values);
    bitonic_sort_uint16_increasing_8v(keys_8, values_8);

    //STEP 1 - Delta = 8
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_8, *values_8);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_14, *values_14);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_7, *values_7, *keys_15, *values_15);

    //STEP 2 - Delta = 4
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_4, *values_4);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_3, *values_3, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_12, *values_12);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_14, *values_14);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_11, *values_11, *keys_15, *values_15);

    //STEP 3 - Delta = 2
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_2, *values_2);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_1, *values_1, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_6, *values_6);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_5, *values_5, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_10, *values_10);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_9, *values_9, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_12, *values_12, *keys_14, *values_14);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_13, *values_13, *keys_15, *values_15);

    //STEP 4 - Delta = 1
    bitonic_sort_uint16_decreasing_compare_vector(*keys, *values, *keys_1, *values_1);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_2, *values_2, *keys_3, *values_3);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_4, *values_4, *keys_5, *values_5);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_6, *values_6, *keys_7, *values_7);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_8, *values_8, *keys_9, *values_9);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_10, *values_10, *keys_11, *values_11);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_12, *values_12, *keys_13, *values_13);
    bitonic_sort_uint16_decreasing_compare_vector(*keys_14, *values_14, *keys_15, *values_15);


    //Final Step : Reorder each Vector
    biotonic_sort_uint16_decreasing_reorder_vector(*keys, *values);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_1, *values_1);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_2, *values_2);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_3, *values_3);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_4, *values_4);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_5, *values_5);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_6, *values_6);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_7, *values_7);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_8, *values_8);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_9, *values_9);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_10, *values_10);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_11, *values_11);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_12, *values_12);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_13, *values_13);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_14, *values_14);
    biotonic_sort_uint16_decreasing_reorder_vector(*keys_15, *values_15);
}

int bitonic_sort_uint16_increasing_hvx(uint16_t* keys, uint16_t* values, uint32_t vectorCount){

    switch(vectorCount){
        case 0:
            return -1; //Nothing to do, error
        case 1:
            bitonic_sort_uint16_increasing_1v(*((HVX_Vector*)keys), *((HVX_Vector*)values));
            return 0;
        case 2:
            bitonic_sort_uint16_increasing_2v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 3:
            bitonic_sort_uint16_increasing_3v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 4:
            bitonic_sort_uint16_increasing_4v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 5:
            bitonic_sort_uint16_increasing_5v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 6:
            bitonic_sort_uint16_increasing_6v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 7:
            bitonic_sort_uint16_increasing_7v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 8:
            bitonic_sort_uint16_increasing_8v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 9:
            bitonic_sort_uint16_increasing_9v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 10:
            bitonic_sort_uint16_increasing_10v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 11:
            bitonic_sort_uint16_increasing_11v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 12:
            bitonic_sort_uint16_increasing_12v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 13:
            bitonic_sort_uint16_increasing_13v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 14:
            bitonic_sort_uint16_increasing_14v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 15:
            bitonic_sort_uint16_increasing_15v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 16:
            bitonic_sort_uint16_increasing_16v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        default:
            return -1; //Not implemented
    };

}
int bitonic_sort_uint16_decreasing_hvx(uint16_t* keys, uint16_t* values, uint32_t vectorCount){
    switch(vectorCount){
        case 0:
            return -1; //Nothing to do, error
        case 1:
            bitonic_sort_uint16_decreasing_1v(*((HVX_Vector*)keys), *((HVX_Vector*)values));
            return 0;
        case 2:
            bitonic_sort_uint16_decreasing_2v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 3:
            bitonic_sort_uint16_decreasing_3v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 4:
            bitonic_sort_uint16_decreasing_4v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 5:
            bitonic_sort_uint16_decreasing_5v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 6:
            bitonic_sort_uint16_decreasing_6v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 7:
            bitonic_sort_uint16_decreasing_7v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 8:
            bitonic_sort_uint16_decreasing_8v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 9:
            bitonic_sort_uint16_decreasing_9v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 10:
            bitonic_sort_uint16_decreasing_10v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 11:
            bitonic_sort_uint16_decreasing_11v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 12:
            bitonic_sort_uint16_decreasing_12v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 13:
            bitonic_sort_uint16_decreasing_13v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 14:
            bitonic_sort_uint16_decreasing_14v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 15:
            bitonic_sort_uint16_decreasing_15v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        case 16:
            bitonic_sort_uint16_decreasing_16v((HVX_Vector*)keys, (HVX_Vector*)values);
            return 0;
        default:
            return -1; //Not implemented
    }
}

//NEEDS A GOOD CHUNK OF MEMORY FOR TESTING (32 vectors)
int bitonic_sort_test(uint16_t* memory){
    uint16_t* keys = memory;
    uint16_t* values = memory + 2*VLEN*16/2;

    for(int i = 0; i < VLEN*16/2; i++){
        keys[i] = (1664525*i + 1013904223) % 65536;
        values[i] = (1664525*i + 1013904223) % 65536;
    }

    bitonic_sort_uint16_increasing_16v((HVX_Vector*)keys, (HVX_Vector*)values);

    for(int i = 0; i < 16; i++){
        for(int j = 0; j < VLEN/2; j++){
            FARF(HIGH, "%d, ", keys[i*VLEN/2 + j]);

        }
        FARF(HIGH, "\n");
    }

    return 0;
}