# Primitive segmentation examples (5 episodes)

Primitives: `reach_to_object` -> `grasp_object` -> `transport_to_target` -> `release_object` -> `return_to_init`.
Frame counts differ per episode; compare segments only within the same episode.

## Rule (auto v1)
- Contiguous `z<=112` mm with length>=2 = low segment.
- First low = grasp_object, second low = release_object.
- Before first low = reach_to_object, between = transport_to_target, after second = return_to_init.
- `target_zone` from place_zone: L/R/M -> left/right/middle.

## Episode 2 (class: `left_to_right` — L→R 예시)
- Total frames: **185** (0..184)
- Low-Z segments: 2
- target_zone: **right**

| primitive | start | end | n_frames | image_start | image_end |
|-----------|------:|----:|---------:|-------------|-----------|
| reach_to_object | 0 | 71 | 72 | frame_000000.jpg | frame_000071.jpg |
| grasp_object | 72 | 77 | 6 | frame_000072.jpg | frame_000077.jpg |
| transport_to_target | 78 | 109 | 32 | frame_000078.jpg | frame_000109.jpg |
| release_object | 110 | 132 | 23 | frame_000110.jpg | frame_000132.jpg |
| return_to_init | 133 | 184 | 52 | frame_000133.jpg | frame_000184.jpg |

```json
{
  "episode": 2,
  "high_level_task": "pick_and_place",
  "object": "red_block",
  "target_zone": "right",
  "episode_class": "left_to_right",
  "segments": [
    {
      "primitive": "reach_to_object",
      "start": 0,
      "end": 71
    },
    {
      "primitive": "grasp_object",
      "start": 72,
      "end": 77
    },
    {
      "primitive": "transport_to_target",
      "start": 78,
      "end": 109
    },
    {
      "primitive": "release_object",
      "start": 110,
      "end": 132
    },
    {
      "primitive": "return_to_init",
      "start": 133,
      "end": 184
    }
  ]
}
```

## Episode 1 (class: `right_to_left` — R→L 예시)
- Total frames: **169** (0..168)
- Low-Z segments: 2
- target_zone: **left**

| primitive | start | end | n_frames | image_start | image_end |
|-----------|------:|----:|---------:|-------------|-----------|
| reach_to_object | 0 | 64 | 65 | frame_000000.jpg | frame_000064.jpg |
| grasp_object | 65 | 80 | 16 | frame_000065.jpg | frame_000080.jpg |
| transport_to_target | 81 | 104 | 24 | frame_000081.jpg | frame_000104.jpg |
| release_object | 105 | 126 | 22 | frame_000105.jpg | frame_000126.jpg |
| return_to_init | 127 | 168 | 42 | frame_000127.jpg | frame_000168.jpg |

```json
{
  "episode": 1,
  "high_level_task": "pick_and_place",
  "object": "red_block",
  "target_zone": "left",
  "episode_class": "right_to_left",
  "segments": [
    {
      "primitive": "reach_to_object",
      "start": 0,
      "end": 64
    },
    {
      "primitive": "grasp_object",
      "start": 65,
      "end": 80
    },
    {
      "primitive": "transport_to_target",
      "start": 81,
      "end": 104
    },
    {
      "primitive": "release_object",
      "start": 105,
      "end": 126
    },
    {
      "primitive": "return_to_init",
      "start": 127,
      "end": 168
    }
  ]
}
```

## Episode 4 (class: `to_middle` — 가운데 place)
- Total frames: **188** (0..187)
- Low-Z segments: 2
- target_zone: **middle**

| primitive | start | end | n_frames | image_start | image_end |
|-----------|------:|----:|---------:|-------------|-----------|
| reach_to_object | 0 | 32 | 33 | frame_000000.jpg | frame_000032.jpg |
| grasp_object | 33 | 46 | 14 | frame_000033.jpg | frame_000046.jpg |
| transport_to_target | 47 | 128 | 82 | frame_000047.jpg | frame_000128.jpg |
| release_object | 129 | 150 | 22 | frame_000129.jpg | frame_000150.jpg |
| return_to_init | 151 | 187 | 37 | frame_000151.jpg | frame_000187.jpg |

```json
{
  "episode": 4,
  "high_level_task": "pick_and_place",
  "object": "red_block",
  "target_zone": "middle",
  "episode_class": "to_middle",
  "segments": [
    {
      "primitive": "reach_to_object",
      "start": 0,
      "end": 32
    },
    {
      "primitive": "grasp_object",
      "start": 33,
      "end": 46
    },
    {
      "primitive": "transport_to_target",
      "start": 47,
      "end": 128
    },
    {
      "primitive": "release_object",
      "start": 129,
      "end": 150
    },
    {
      "primitive": "return_to_init",
      "start": 151,
      "end": 187
    }
  ]
}
```

## Episode 5 (class: `M_to_L`)
- Total frames: **165** (0..164)
- Low-Z segments: 2
- target_zone: **left**

| primitive | start | end | n_frames | image_start | image_end |
|-----------|------:|----:|---------:|-------------|-----------|
| reach_to_object | 0 | 47 | 48 | frame_000000.jpg | frame_000047.jpg |
| grasp_object | 48 | 53 | 6 | frame_000048.jpg | frame_000053.jpg |
| transport_to_target | 54 | 79 | 26 | frame_000054.jpg | frame_000079.jpg |
| release_object | 80 | 100 | 21 | frame_000080.jpg | frame_000100.jpg |
| return_to_init | 101 | 164 | 64 | frame_000101.jpg | frame_000164.jpg |

```json
{
  "episode": 5,
  "high_level_task": "pick_and_place",
  "object": "red_block",
  "target_zone": "left",
  "episode_class": "M_to_L",
  "segments": [
    {
      "primitive": "reach_to_object",
      "start": 0,
      "end": 47
    },
    {
      "primitive": "grasp_object",
      "start": 48,
      "end": 53
    },
    {
      "primitive": "transport_to_target",
      "start": 54,
      "end": 79
    },
    {
      "primitive": "release_object",
      "start": 80,
      "end": 100
    },
    {
      "primitive": "return_to_init",
      "start": 101,
      "end": 164
    }
  ]
}
```

## Episode 23 (class: `M_to_R`)
- Total frames: **158** (0..157)
- Low-Z segments: 2
- target_zone: **right**

| primitive | start | end | n_frames | image_start | image_end |
|-----------|------:|----:|---------:|-------------|-----------|
| reach_to_object | 0 | 61 | 62 | frame_000000.jpg | frame_000061.jpg |
| grasp_object | 62 | 69 | 8 | frame_000062.jpg | frame_000069.jpg |
| transport_to_target | 70 | 111 | 42 | frame_000070.jpg | frame_000111.jpg |
| release_object | 112 | 131 | 20 | frame_000112.jpg | frame_000131.jpg |
| return_to_init | 132 | 157 | 26 | frame_000132.jpg | frame_000157.jpg |

```json
{
  "episode": 23,
  "high_level_task": "pick_and_place",
  "object": "red_block",
  "target_zone": "right",
  "episode_class": "M_to_R",
  "segments": [
    {
      "primitive": "reach_to_object",
      "start": 0,
      "end": 61
    },
    {
      "primitive": "grasp_object",
      "start": 62,
      "end": 69
    },
    {
      "primitive": "transport_to_target",
      "start": 70,
      "end": 111
    },
    {
      "primitive": "release_object",
      "start": 112,
      "end": 131
    },
    {
      "primitive": "return_to_init",
      "start": 132,
      "end": 157
    }
  ]
}
```

---

See also `PRIMITIVE_SEGMENTATION_SPEC.md`.