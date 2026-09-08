# Positioning

1. Move to base position

```bash
jaka-cli restore
```

2. Set base position

```bash
jaka-cli set-base
```

# After Positioning

Height references (relative to base position):

High position: z = 100

```bash
jaka-cli move-to --speed=2000 --rel 0 0 100
```

Low position: z = -136

```bash
jaka-cli move-to --speed=2000 --rel 0 0 -136
```

Cube height: z = -125

```bash
jaka-cli move-to --speed=2000 --rel 0 0 -125
```

Position references (relative to base position):

Cube placement area: 360 -200 0

```bash
jaka-cli move-to --speed=2000 --rel 360 -200 0
```

# Suction Cup

Enable

```bash
jaka-cli do tool 0 on && jaka-cli do tool 1 on
```

Disable

```bash
jaka-cli do tool 0 off && jaka-cli do tool 1 off
```
