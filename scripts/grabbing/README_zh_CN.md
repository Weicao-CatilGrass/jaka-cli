# 定位

1. 移动到基准位置

```bash
jaka-cli restore
```

2. 设置基准位置

```bash
jaka-cli set-base
```

# 定位后

高度参考（相对于基准位置）：

高位: z = 100

```bash
jaka-cli move-to --speed=2000 --rel 0 0 100
```

低位: z = -136

```bash
jaka-cli move-to --speed=2000 --rel 0 0 -136
```

方块高度: z = -125

```bash
jaka-cli move-to --speed=2000 --rel 0 0 -125
```

位置参考（相对于基准位置）：

方块放置区域：360 -200 0

```bash
jaka-cli move-to --speed=2000 --rel 360 -200 0
```

# 吸盘

启用

```bash
jaka-cli do tool 0 on && jaka-cli do tool 1 on
```

关闭

```bash
jaka-cli do tool 0 off && jaka-cli do tool 1 off
```
