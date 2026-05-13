# Topology comparison — five scenarios, N=60

Empirical comparison of the four PSO outer-loop topologies (`gbest`,
`niched`, `ring`, `von_neumann`) on the tuning-study scenario battery,
under two coefficient regimes:

1. **Default** (`SearchConfig::default()`): `inertia=0.2`, `c1=1.6`,
   `c2=0.85`.
2. **Textbook** (Clerc–Kennedy): `inertia=0.328`, `c1=1.98`, `c2=1.99`.

Each (scenario, topology, regime) cell is 5 runs at seeds 1–5;
per-cell aggregates are mean / best / worst / spread / mean search
time. Higher fitness is better (the score is negative cost — closer
to zero wins). All runs at **N=60 waypoints** so the comparison is
apples-to-apples across scenarios and regimes.

---

## Methodology

**Run command:**

```sh
bywind-cli search \
    --load-baked profiling/tuning/<scenario>/baked.bk1 \
    --config    profiling/tuning/_search.toml \
    --config    profiling/tuning/<scenario>/config.toml \
    [--config   target/textbook.toml]   # textbook regime only
    --waypoints 60 \
    --topology  {gbest,niched,ring,von_neumann} \
    --seed      {1..5} \
    --out       NUL
```

**Effective `SearchSettings` (default regime):**

| Field | Value | Source |
|---|---:|---|
| `particle_count_space` | 40 | `_search.toml` |
| `particle_count_time` | 40 | `_search.toml` |
| `max_iteration_space` | 40 | `_search.toml` |
| `max_iteration_time` | 30 | `_search.toml` |
| `inertia` | 0.2 | `SearchConfig::default()` |
| `cognitive_coeff` | 1.6 | `SearchConfig::default()` |
| `social_coeff` | 0.85 | `SearchConfig::default()` |
| `path_kick_probability` | 0.1 | `SearchConfig::default()` |
| `path_kick_gamma_0_fraction` | 0.05 | `SearchConfig::default()` |
| `path_kick_gamma_min_fraction` | 0.005 | `SearchConfig::default()` |
| `mutation_gamma_*_fraction` | 0.0 (disabled) | `SearchSettings::default()` |
| `init_shares` | `InitShares::default()` | `SearchSettings::default()` |
| `baseline_shares` | 40% north / 40% south / 20% straight | `BaselineShares::default()` |
| `waypoints` | 60 | CLI `--waypoints 60` |

**Textbook regime overrides:** `inertia=0.328`, `cognitive_coeff=1.98`,
`social_coeff=1.99` (via `target/textbook.toml`, layered on top of the
two scenario configs).

---

## Default PSO params, N=60

### short-easy (Boston → Halifax, ~600 km, open Atlantic)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **von_neumann** | -712,120 | -704,128 | -717,484 | 13,356 | 1.21 | **-0.68%** (better) |
| gbest | -716,971 | -708,608 | -721,491 | 12,883 | 1.20 | baseline |
| ring | -718,962 | -711,684 | -726,386 | 14,702 | 1.24 | +0.28% worse |
| niched | -720,114 | -714,644 | -730,901 | 16,257 | 1.27 | +0.44% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-720993.3202,1.20
gbest,2,-708607.9646,1.17
gbest,3,-721491.2073,1.23
gbest,4,-713072.1632,1.20
gbest,5,-720692.1918,1.22
niched,1,-730901.1068,1.27
niched,2,-715127.5783,1.30
niched,3,-714643.7138,1.25
niched,4,-723337.0855,1.27
niched,5,-716560.1151,1.27
ring,1,-713878.3141,1.24
ring,2,-711683.5143,1.25
ring,3,-721239.8134,1.22
ring,4,-721623.4426,1.26
ring,5,-726385.7779,1.25
von_neumann,1,-716062.0112,1.21
von_neumann,2,-709610.6685,1.21
von_neumann,3,-713316.8693,1.20
von_neumann,4,-717483.8577,1.22
von_neumann,5,-704128.1204,1.22
```

### archipelago (Athens → Antalya, ~1100 km, weaves through Aegean islands)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **von_neumann** | -158,492 | -152,779 | -166,778 | 13,999 | 1.17 | **-1.79%** (better) |
| gbest | -161,378 | -155,185 | -170,223 | 15,038 | 1.16 | baseline |
| ring | -163,450 | -155,459 | -169,763 | 14,303 | 1.21 | +1.28% worse |
| niched | -163,557 | -158,397 | -170,223 | 11,826 | 1.24 | +1.35% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-155184.6394,1.10
gbest,2,-170223.0359,1.23
gbest,3,-158396.6519,1.15
gbest,4,-158359.7002,1.16
gbest,5,-164725.2161,1.17
niched,1,-160018.0881,1.19
niched,2,-170223.0359,1.30
niched,3,-158396.6519,1.22
niched,4,-164176.8213,1.24
niched,5,-164969.5263,1.23
ring,1,-158475.0358,1.20
ring,2,-169282.8332,1.23
ring,3,-155459.3628,1.19
ring,4,-164272.3310,1.21
ring,5,-169762.6393,1.20
von_neumann,1,-152896.6059,1.15
von_neumann,2,-163376.0463,1.19
von_neumann,3,-152779.3044,1.15
von_neumann,4,-156628.7500,1.14
von_neumann,5,-166777.8587,1.21
```

### coastal-detour (NYC → New Orleans, ~3000 km, around Florida)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **von_neumann** | -4,263,730 | -4,228,764 | -4,284,165 | 55,401 | 1.53 | **-1.47%** (better) |
| ring | -4,308,448 | -4,268,490 | -4,360,129 | 91,639 | 1.58 | -0.43% (better) |
| gbest | -4,327,233 | -4,250,621 | -4,416,031 | 165,410 | 1.53 | baseline |
| niched | -4,372,225 | -4,233,443 | -4,440,276 | 206,833 | 1.59 | +1.04% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-4291664.0934,1.53
gbest,2,-4250620.7076,1.54
gbest,3,-4416030.8234,1.51
gbest,4,-4281728.2455,1.57
gbest,5,-4396121.9802,1.51
niched,1,-4233442.9381,1.55
niched,2,-4354496.1912,1.62
niched,3,-4412178.9489,1.58
niched,4,-4440276.2989,1.58
niched,5,-4420728.2179,1.62
ring,1,-4268489.8086,1.57
ring,2,-4282206.2569,1.59
ring,3,-4272076.1026,1.58
ring,4,-4360128.9102,1.56
ring,5,-4359341.0242,1.58
von_neumann,1,-4251405.6538,1.52
von_neumann,2,-4270659.0978,1.55
von_neumann,3,-4228764.1014,1.56
von_neumann,4,-4284164.7187,1.52
von_neumann,5,-4283657.6204,1.52
```

### transoceanic-pacific (San Francisco → Tokyo, ~9000 km, antimeridian crossing)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **von_neumann** | -2,320,045 | -2,299,431 | -2,329,530 | 30,099 | 6.31 | **-0.14%** (better) |
| gbest | -2,323,325 | -2,306,126 | -2,355,188 | 49,062 | 5.34 | baseline |
| niched | -2,324,382 | **-2,282,188** | -2,354,234 | 72,047 | 5.66 | +0.05% (≈ tie) |
| ring | -2,325,771 | -2,305,567 | -2,343,120 | 37,553 | 5.62 | +0.11% (≈ tie) |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-2355188.2201,5.04
gbest,2,-2317832.4097,5.49
gbest,3,-2313334.6798,5.05
gbest,4,-2306126.0758,5.37
gbest,5,-2324145.1703,5.74
niched,1,-2282187.8248,5.27
niched,2,-2354234.4649,5.66
niched,3,-2341687.9928,5.48
niched,4,-2306127.5784,5.70
niched,5,-2337670.5303,6.20
ring,1,-2305566.7880,5.22
ring,2,-2327947.7127,5.54
ring,3,-2318001.5103,5.33
ring,4,-2334219.4587,5.77
ring,5,-2343119.7149,6.23
von_neumann,1,-2329529.7989,5.61
von_neumann,2,-2329010.4897,5.58
von_neumann,3,-2317327.6208,8.76
von_neumann,4,-2299430.5117,5.47
von_neumann,5,-2324928.2392,6.15
```

### transoceanic-atlantic (Lisbon → Buenos Aires, ~10000 km, both hemispheres)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **von_neumann** | -2,149,358 | -2,131,952 | -2,173,286 | 41,333 | 1.52 | **-0.94%** (better) |
| **niched** | -2,159,648 | -2,150,347 | -2,173,640 | 23,293 | 1.63 | **-0.46%** (better) |
| ring | -2,167,696 | -2,143,832 | -2,187,770 | 43,937 | 1.56 | -0.09% (≈ tie) |
| gbest | -2,169,691 | -2,149,899 | -2,186,747 | 36,848 | 1.64 | baseline |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-2186746.8032,1.65
gbest,2,-2178347.5349,1.67
gbest,3,-2174255.1666,1.61
gbest,4,-2159208.0172,1.70
gbest,5,-2149898.6363,1.56
niched,1,-2173639.8426,1.67
niched,2,-2158610.3037,1.67
niched,3,-2150346.8373,1.61
niched,4,-2163352.7696,1.59
niched,5,-2152291.2165,1.61
ring,1,-2164457.8994,1.57
ring,2,-2164599.3952,1.57
ring,3,-2177818.8947,1.55
ring,4,-2187769.5560,1.56
ring,5,-2143832.0783,1.55
von_neumann,1,-2138253.1897,1.51
von_neumann,2,-2131952.2349,1.53
von_neumann,3,-2144254.0723,1.52
von_neumann,4,-2173285.5366,1.54
von_neumann,5,-2159046.6472,1.52
```

### Rollup — default PSO params at N=60 (vs gbest, % cost-delta)

| | short-easy | archipelago | coastal-detour | trans-pac | trans-atl |
|---|---:|---:|---:|---:|---:|
| von_neumann | **-0.68%** | **-1.79%** | **-1.47%** | **-0.14%** | **-0.94%** |
| ring | +0.28% | +1.28% | -0.43% | +0.11% | -0.09% |
| niched | +0.44% | +1.35% | +1.04% | +0.05% | **-0.46%** |

**Headline observations (default params):**

- **Von Neumann wins on every single scenario.** Margin from -0.14% (trans-pacific, ≈ noise) to -1.79% (archipelago). The 4-neighbour torus is the most consistent topology under the gentle default coefficients.
- **Ring is mid-pack.** Slight wins on coastal-detour and trans-atlantic, slight losses on short-easy and archipelago. Always within ~1.3% of gbest.
- **Niched only beats gbest on trans-atlantic** (-0.46%). On the medium scenarios it loses by ~1% — its per-niche reduction overhead isn't paid back by corridor diversity that doesn't really exist (or is dominated by single sea routes around obstacles).

---

## Textbook PSO params, N=60

### short-easy (Boston → Halifax)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **gbest** | -720,293 | -705,599 | -737,802 | 32,203 | 1.94 | baseline |
| von_neumann | -729,158 | -710,835 | -738,754 | 27,919 | 2.03 | +1.23% worse |
| niched | -731,963 | -714,321 | -739,401 | 25,080 | 2.26 | +1.62% worse |
| ring | -737,354 | -718,688 | -752,638 | 33,951 | 2.07 | +2.37% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-737802.1811,1.94
gbest,2,-714321.0601,2.01
gbest,3,-716212.1796,2.02
gbest,4,-727532.4711,1.97
gbest,5,-705599.0777,1.78
niched,1,-737964.2629,2.18
niched,2,-714321.0601,2.21
niched,3,-739401.4129,2.49
niched,4,-735360.2089,2.30
niched,5,-732770.3349,2.10
ring,1,-746092.1516,2.08
ring,2,-718687.6513,2.04
ring,3,-744092.1943,2.12
ring,4,-752638.2843,2.08
ring,5,-725258.1253,2.01
von_neumann,1,-725510.2247,2.07
von_neumann,2,-710835.0651,2.00
von_neumann,3,-738753.6369,2.03
von_neumann,4,-736229.4035,2.12
von_neumann,5,-734459.3124,1.92
```

### archipelago (Athens → Antalya)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **von_neumann** | -156,571 | -151,141 | -161,465 | 10,324 | 1.88 | **-1.83%** (better) |
| gbest | -159,498 | -153,483 | -169,757 | 16,273 | 1.83 | baseline |
| ring | -159,959 | -155,590 | -168,233 | 12,643 | 2.05 | +0.29% worse |
| niched | -162,515 | -155,342 | -172,164 | 16,822 | 2.19 | +1.89% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-153483.1824,1.69
gbest,2,-169756.5721,1.94
gbest,3,-155886.8665,1.84
gbest,4,-157336.1676,1.93
gbest,5,-161024.7428,1.74
niched,1,-155341.5557,2.18
niched,2,-172163.7824,2.38
niched,3,-155886.8665,2.18
niched,4,-163461.7777,2.15
niched,5,-165723.1074,2.07
ring,1,-155589.9555,1.97
ring,2,-168232.6098,2.12
ring,3,-157444.7999,2.05
ring,4,-158716.7058,2.08
ring,5,-159809.8356,2.02
von_neumann,1,-151141.2290,1.77
von_neumann,2,-159719.1874,1.98
von_neumann,3,-157951.8557,2.02
von_neumann,4,-152578.5800,1.85
von_neumann,5,-161465.2376,1.77
```

### coastal-detour (NYC → New Orleans)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **niched** | -4,308,644 | -4,230,115 | -4,432,887 | 202,772 | 2.48 | **-0.03%** (≈ tie) |
| gbest | -4,309,774 | -4,242,560 | -4,440,669 | 198,110 | 2.11 | baseline |
| von_neumann | -4,318,542 | -4,278,725 | -4,382,186 | 103,461 | 2.15 | +0.20% worse |
| ring | -4,376,701 | -4,316,275 | -4,419,971 | 103,697 | 2.37 | +1.55% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-4440669.1685,2.15
gbest,2,-4308038.4780,2.13
gbest,3,-4294446.8228,2.19
gbest,4,-4242559.6401,2.08
gbest,5,-4263153.5897,2.02
niched,1,-4432886.7067,2.57
niched,2,-4249777.9271,2.68
niched,3,-4310703.5391,2.50
niched,4,-4319735.6879,2.51
niched,5,-4230114.9039,2.16
ring,1,-4404673.8299,2.26
ring,2,-4419971.1470,2.32
ring,3,-4355618.8333,2.46
ring,4,-4386965.5011,2.42
ring,5,-4316274.5876,2.38
von_neumann,1,-4278724.8420,2.07
von_neumann,2,-4332865.3431,2.21
von_neumann,3,-4382185.5936,2.17
von_neumann,4,-4296858.7282,2.20
von_neumann,5,-4302073.6639,2.09
```

### transoceanic-pacific (San Francisco → Tokyo)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **niched** | -2,294,334 | **-2,187,500** | -2,376,714 | 189,214 | 7.83 | **-0.23%** (better) |
| von_neumann | -2,294,752 | -2,264,740 | -2,338,436 | 73,697 | 6.72 | -0.22% (better) |
| gbest | -2,299,733 | -2,187,500 | -2,400,668 | 213,168 | 6.48 | baseline |
| ring | -2,303,099 | -2,244,242 | -2,375,009 | 130,768 | 6.72 | +0.15% (≈ tie) |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-2187500.4749,5.14
gbest,2,-2400668.2044,6.08
gbest,3,-2279027.6719,6.01
gbest,4,-2307942.2984,9.21
gbest,5,-2323528.7831,5.95
niched,1,-2187500.4749,5.71
niched,2,-2376714.4462,10.07
niched,3,-2289851.2909,6.86
niched,4,-2320975.5225,8.03
niched,5,-2296628.0727,8.47
ring,1,-2325650.4107,6.02
ring,2,-2244241.6634,6.28
ring,3,-2282080.0654,6.73
ring,4,-2288511.9345,6.62
ring,5,-2375009.4153,7.96
von_neumann,1,-2264739.5797,6.13
von_neumann,2,-2274680.3717,6.28
von_neumann,3,-2318562.1825,7.08
von_neumann,4,-2338436.1184,6.04
von_neumann,5,-2277340.5872,8.09
```

### transoceanic-atlantic (Lisbon → Buenos Aires)

| Topology | Mean fit | Best | Worst | Spread | Mean search (s) | vs gbest |
|---|---:|---:|---:|---:|---:|---:|
| **niched** | -2,113,962 | -2,085,603 | -2,149,286 | 63,683 | 2.31 | **-0.22%** (better) |
| gbest | -2,118,545 | -2,085,168 | -2,158,760 | 73,592 | 2.06 | baseline |
| von_neumann | -2,127,725 | -2,112,953 | -2,143,051 | 30,098 | 2.10 | +0.43% worse |
| ring | -2,134,460 | -2,121,890 | -2,148,994 | 27,104 | 2.26 | +0.75% worse |

#### Raw runs

```
topology,seed,fitness,search_seconds
gbest,1,-2138590.0729,1.98
gbest,2,-2158759.7245,2.05
gbest,3,-2108915.1500,2.14
gbest,4,-2101292.8915,2.14
gbest,5,-2085168.1633,2.00
niched,1,-2126334.3069,2.17
niched,2,-2085602.5244,2.23
niched,3,-2108915.1500,2.56
niched,4,-2149285.8169,2.40
niched,5,-2099673.6466,2.20
ring,1,-2124500.3878,2.05
ring,2,-2121890.1545,2.26
ring,3,-2138896.6164,2.36
ring,4,-2148994.3908,2.33
ring,5,-2138017.0675,2.29
von_neumann,1,-2120243.7102,1.97
von_neumann,2,-2135431.9178,2.14
von_neumann,3,-2112953.2144,2.12
von_neumann,4,-2143050.7392,2.12
von_neumann,5,-2126943.5939,2.17
```

### Rollup — textbook PSO params at N=60 (vs gbest, % cost-delta)

| | short-easy | archipelago | coastal-detour | trans-pac | trans-atl |
|---|---:|---:|---:|---:|---:|
| von_neumann | +1.23% | **-1.83%** | +0.20% | -0.22% | +0.43% |
| ring | +2.37% | +0.29% | +1.55% | +0.15% | +0.75% |
| niched | +1.62% | +1.89% | **-0.03%** | **-0.23%** | **-0.22%** |

**Headline observations (textbook params):**

- **Niched is the most consistent winner** under textbook coefficients —
  best on 3 of 5 scenarios. Wins are smaller in magnitude than VN's
  default-regime wins (~0.2% vs ~1%), but more uniformly distributed.
- **VN's advantage shifted.** Still has the largest single win (-1.83%
  on archipelago) but loses on the long routes where it dominated
  under defaults. On 3 of 5 scenarios it's *worse* than gbest under
  textbook params, vs winning everywhere under defaults.
- **Ring loses on every scenario** under textbook params.
- **Topology effects are larger** under textbook params — the spread
  between best and worst topology in each column is wider than under
  defaults.

---

## Side-by-side: default vs textbook (vs gbest, both at N=60)

| Topology | Regime | short-easy | archipelago | coastal-detour | trans-pac | trans-atl |
|---|---|---:|---:|---:|---:|---:|
| von_neumann | default | -0.68% | -1.79% | -1.47% | -0.14% | -0.94% |
| von_neumann | textbook | +1.23% | -1.83% | +0.20% | -0.22% | +0.43% |
| ring | default | +0.28% | +1.28% | -0.43% | +0.11% | -0.09% |
| ring | textbook | +2.37% | +0.29% | +1.55% | +0.15% | +0.75% |
| niched | default | +0.44% | +1.35% | +1.04% | +0.05% | -0.46% |
| niched | textbook | +1.62% | +1.89% | -0.03% | -0.23% | -0.22% |

### Hypothesis: why textbook params reshuffle the topology ranking

The textbook params have substantially stronger social pull than the
defaults — `social_coeff` jumps from 0.85 to 1.99, more than 2×. That
matters topology-by-topology:

- **gbest**: faster collapse onto the global best. Good when the
  landscape is uni-modal (short-easy, where gbest wins outright);
  costly when it's multi-modal (the long routes, where gbest still
  loses to niched).
- **VN / ring**: the social pull is toward the *local* best among
  neighbours. Stronger pull → faster local collapse → less of the
  diversity preservation that lbest is supposed to provide. The lbest
  variants effectively "lose their personality" under textbook params,
  which is why VN's default-regime sweep collapses to per-scenario
  contention.
- **Niched**: each niche has its own social attractor. Stronger pull
  refines *within* a niche but cannot leak across niches by
  construction. Niched's diversity preservation lives in the
  partition, not in the social-pull weighting, so it's the only
  topology whose corridor preservation survives strong textbook
  social pull.

### gbest baseline: default vs textbook

For reference: how do the textbook params compare to defaults on the
gbest baseline? Both at N=60 now, so this is apples-to-apples.

| Scenario | default gbest | textbook gbest | textbook delta |
|---|---:|---:|---:|
| short-easy | -716,971 | -720,293 | +0.46% (default better) |
| archipelago | -161,378 | -159,498 | -1.16% (textbook better) |
| coastal-detour | -4,327,233 | -4,309,774 | -0.40% (textbook better) |
| trans-pacific | -2,323,325 | -2,299,733 | -1.02% (textbook better) |
| trans-atlantic | -2,169,691 | -2,118,545 | -2.36% (textbook better) |

Textbook gbest beats default gbest on 4 of 5 scenarios. Short-easy is
the exception — its single uni-modal basin means the more aggressive
textbook coefficients overshoot. The advantage grows with route
length / multi-modality (largest on trans-atlantic at -2.36%).

### Cross-regime: VN-default vs gbest-textbook

Each regime has a within-regime winner — VN under defaults, gbest
under textbook (for short-easy) / niched under textbook (elsewhere).
The natural cross-regime question: how does default's best-non-gbest
topology stack up against textbook's best-baseline topology?

| Scenario | VN-default | gbest-textbook | Better config | Margin |
|---|---:|---:|---|---:|
| short-easy | -712,120 | -720,293 | **VN-default** | -1.13% |
| archipelago | -158,492 | -159,498 | **VN-default** | -0.63% |
| coastal-detour | -4,263,730 | -4,309,774 | **VN-default** | -1.07% |
| trans-pacific | -2,320,045 | -2,299,733 | **gbest-textbook** | +0.88% |
| trans-atlantic | -2,149,358 | -2,118,545 | **gbest-textbook** | +1.45% |

Neither config dominates. **VN-default wins on the shorter / medium
routes** (≤ 3000 km in this battery); **gbest-textbook wins on the
long transoceanic routes** (≥ 9000 km). The crossover lives somewhere
in the gap between coastal-detour (3000 km) and trans-pacific
(9000 km) — this study doesn't have a scenario in between to localise
it more precisely.

Mechanism: VN-default's "play it safe" combo (gentle coefficients,
4-neighbour torus) gives particles time to explore neighbour-best
basins on routes where the optimum is reachable from the init
population. gbest-textbook's "commit hard" combo (aggressive
coefficients, single global attractor) collapses everyone fast — wins
on long routes where the per-dimension iteration budget is starved
(60 waypoints × 2 axes ÷ 40 iters = ~3 iters per coordinate, far
below the literature's recommended ~10×) and any iter not pulling
toward the leader is wasted.

### Best configuration per scenario across all eight (default × 4 + textbook × 4)

| Scenario | Winner | Mean fit | vs scenario's gbest-default | vs scenario's gbest-textbook |
|---|---|---:|---:|---:|
| short-easy | VN-default | -712,120 | -0.68% | -1.13% |
| archipelago | VN-textbook | -156,571 | -2.98% | -1.83% |
| coastal-detour | VN-default | -4,263,730 | -1.47% | -1.07% |
| trans-pacific | niched-textbook | -2,294,334 | -1.25% | -0.23% |
| trans-atlantic | niched-textbook | -2,113,962 | -2.57% | -0.22% |

The route-length-aware best policy:

- **≤ ~3000 km**: VN-default (3 of 5 scenarios; archipelago fell to
  VN-textbook by a hair, but its many-small-obstacle structure is
  unusual).
- **≥ ~9000 km**: niched-textbook.

If you can only ship one configuration: **textbook-gbest is the safer
default** because its losses to VN-default on the short/medium routes
are smaller (1.13% worst) than VN-default's losses on the long routes
(1.45% worst), and the long routes are the higher-stakes case for
real users (more compute spent, the optimization actually matters).

---

## Practical takeaway

| | Default coefficients | Textbook coefficients |
|---|---|---|
| **Topology to pick** | von_neumann | niched (with gbest fallback for short uni-modal routes) |
| **Why** | gentle social pull → neighbour-graph diversity (VN) is strongest | aggressive social pull → only partition-based diversity (niched) survives |
| **Expected gain over gbest** | ~0.5–1.5% across the board | ~0.2% on the multi-modal long routes, no gain elsewhere |

**Search-time penalty** for non-gbest topologies stays at ~5–10%
across both regimes; niched is consistently slowest, von_neumann
barely above gbest.

The bigger overall lever in this study is the **coefficient choice
itself** (textbook gbest beats default gbest by 1–2% on the long
routes), not the topology. The topology choice gives ~1% on top of
that on the matching regime. Both levers compose in the obvious way:
the best single configuration on the long multi-modal routes is
**textbook coefficients + niched topology**.
