# Polygon Editor

`polygon_editor.py` 是给 `rmuc_2026` 区域参数做可视化编辑的小工具。

它支持：

- 同时显示多个区域
- 使用 `Publish Point` 加点
- 使用 `Interact` 拖动顶点微调
- 实时输出 YAML 字符串
- 切换当前激活区域
- 撤销、清空、导出

脚本位置：

- [polygon_editor.py](/home/mihu/hj_sentry_behavior/src/hj_sentry_behavior/scripts/polygon_editor.py)

## 1. 启动

直接用 Python 启动，不依赖 `ros2 run`：

```bash
python3 /home/mihu/hj_sentry_behavior/src/hj_sentry_behavior/scripts/polygon_editor.py \
  --ros-args \
  -p polygon_names:="[supply_our_half_polygon, rough_road_polygon, highland_polygon]" \
  -p polygon_strings:="[\"8.798,4.567;5.648,-0.036;5.610,-1.430;6.758,-4.367;-3.660,-4.329;-3.632,1.631;3.980,1.861;6.382,5.331;8.557,5.384\", \"7.038,-6.356;7.076,-8.283;0.121,-8.330;0.106,-5.781;1.528,-5.804;1.572,-6.339\", \"14.335,4.353;14.102,-2.893;11.640,-6.176;9.483,-6.189;9.500,-8.365;6.898,-8.339;6.579,0.854;8.942,4.304;12.047,4.467\"]" \
  -p active_polygon:=supply_our_half_polygon
```

如果只想编辑一个区域，也可以只给一组参数：

```bash
python3 /home/mihu/hj_sentry_behavior/src/hj_sentry_behavior/scripts/polygon_editor.py \
  --ros-args \
  -p polygon_name:=supply_our_half_polygon \
  -p initial_polygon:='8.798,4.567;5.648,-0.036;5.610,-1.430'
```

python3 /home/mihu/hj_sentry_behavior/src/hj_sentry_behavior/scripts/polygon_editor.py \
  --ros-args \
  -p polygon_name:=rough_road_polygon \
  -p initial_polygon:='7.038,-6.356;7.076,-8.283;0.121,-8.330;0.106,-5.781;1.528,-5.804;1.572,-6.339'

## 2. RViz 配置

至少加这几个：

- `Publish Point`
- `MarkerArray`
- `InteractiveMarkers`

对应 topic：

- `Publish Point`：发到 `/clicked_point`
- `MarkerArray`：订阅 `/polygon_editor/markers`
- `InteractiveMarkers`：用 `/polygon_editor/interactive`

显示规则：

- 每个区域一种颜色
- 当前激活区域更亮、更粗，标题后面带 `[active]`
- 所有顶点都可以拖动

## 3. 基本操作

### 3.1 加点

1. 在 RViz 里切到 `Publish Point`
2. 在地图上点一下
3. 新点会加入当前激活区域
4. 终端会打印更新后的 YAML

### 3.2 拖动微调

1. 在 RViz 里切到 `Interact`
2. 鼠标点住某个顶点球
3. 直接拖动
4. 松手后终端会打印新的 YAML

交互点支持：

- 平面拖动
- X 轴拖动
- Y 轴拖动

一般直接平面拖就够了。

### 3.3 切换当前编辑区域

```bash
ros2 param set /polygon_editor active_polygon supply_our_half_polygon
ros2 param set /polygon_editor active_polygon rough_road_polygon
ros2 param set /polygon_editor active_polygon highland_polygon
```

切换之后：

- `Publish Point` 新加的点会进这个区域
- 这个区域会高亮显示

## 4. 服务接口

撤销当前激活区域最后一个点：

```bash
ros2 service call /polygon_editor/undo_last std_srvs/srv/Trigger {}
```

清空当前激活区域：

```bash
ros2 service call /polygon_editor/clear std_srvs/srv/Trigger {}
```

打印当前全部区域 YAML：

```bash
ros2 service call /polygon_editor/print_yaml std_srvs/srv/Trigger {}
```

## 5. 终端输出说明

每次加点、拖点、切区后，终端会打印类似内容：

```yaml
supply_our_half_polygon: "8.798,4.567;5.648,-0.036;..."
rough_road_polygon: "7.038,-6.356;7.076,-8.283;..."
highland_polygon: "14.335,4.353;14.102,-2.893;..."
```

把这几行直接贴回 [sentry_behavior.yaml](/home/mihu/hj_sentry_behavior/src/hj_sentry_behavior/params/sentry_behavior.yaml) 里对应参数即可。

## 6. 参数说明

常用参数：

- `polygon_names`
  多个区域名列表
- `polygon_strings`
  与 `polygon_names` 一一对应的 polygon 字符串
- `active_polygon`
  当前激活区域
- `frame_id`
  默认是 `map`
- `clicked_point_topic`
  默认是 `/clicked_point`
- `marker_topic`
  默认是 `/polygon_editor/markers`
- `precision`
  输出小数位数，默认 `3`

polygon 字符串格式：

```text
x1,y1;x2,y2;x3,y3;...
```

注意：

- 点按边界顺序排
- 顺时针或逆时针都可以
- 不要自交
- 不需要重复第一个点

## 7. 使用建议

推荐流程：

1. 先把 3 个区域都载入
2. 同屏看边界有没有重合或留缝
3. 切换 `active_polygon`
4. 用 `Interact` 拖点修边界
5. 不够用时再用 `Publish Point` 补点
6. 最后调用 `print_yaml`
7. 把输出写回参数文件

如果边界附近容易出现“既不在 A 也不在 B”的情况，建议：

- 不要留缝
- 可接受轻微重叠
- 再结合行为树里的优先级决定覆盖关系

## 8. 常见问题

### 点地图没反应

通常是因为当前工具还停在 `Interact`，没有切到 `Publish Point`。

### 顶点拖不动

通常是因为当前工具还停在 `Publish Point`，没有切到 `Interact`。

### 只想改某一个区域

切换：

```bash
ros2 param set /polygon_editor active_polygon <区域名>
```

然后再点或拖。

### 命令行启动报错，提示 `can't find '__main__' module`

一般是因为把脚本路径断行断坏了，确保这一段是完整的：

```bash
python3 /home/mihu/hj_sentry_behavior/src/hj_sentry_behavior/scripts/polygon_editor.py
```

不要把 `scripts/` 和 `polygon_editor.py` 拆成两行。
