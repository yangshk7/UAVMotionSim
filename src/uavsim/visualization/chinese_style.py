"""为项目图表选取已安装的中文字体，不改变数值数据。"""

from matplotlib import font_manager, rcParams


def use_chinese_font() -> str:
    """配置 Matplotlib 中文字体；无可用字体时保持默认并返回字体名。"""
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "SimSun"):
        if name in available:
            rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            rcParams["axes.unicode_minus"] = False
            return name
    return rcParams["font.sans-serif"][0]
