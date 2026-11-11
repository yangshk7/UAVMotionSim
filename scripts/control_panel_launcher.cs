// Project-local Windows launcher. Build with build_control_panel_exe.ps1.
using System;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;
using System.Reflection;

[assembly: AssemblyVersion("1.1.0.0")]
[assembly: AssemblyFileVersion("1.1.0.0")]
[assembly: AssemblyProduct("UAVMotionSim")]

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        string main = AppDomain.CurrentDomain.BaseDirectory;
        string project = Path.GetFullPath(Path.Combine(main, ".."));
        string python = Path.Combine(project, ".venv", "Scripts", "pythonw.exe");
        string panel = Path.Combine(main, "internal", "control_panel.py");
        try
        {
            if (!File.Exists(python) || !File.Exists(panel))
                throw new FileNotFoundException("请将 EXE 保留在项目的 main 文件夹内，并先运行 scripts/setup.ps1 安装项目环境。");
            // --check verifies the installed launcher without opening another window.
            if (args.Length == 1 && args[0] == "--check") return 0;
            ProcessStartInfo start = new ProcessStartInfo(python);
            start.Arguments = "\"" + panel + "\"";
            start.WorkingDirectory = project;
            start.UseShellExecute = false;
            start.CreateNoWindow = true;
            start.EnvironmentVariables["PYTHONUTF8"] = "1";
            Process child = Process.Start(start);
            if (child == null) throw new InvalidOperationException("无法启动上位机。");
            if (child.WaitForExit(2000) && child.ExitCode != 0)
                throw new InvalidOperationException("上位机启动失败，请查看 main/gui_logs 中最新的 startup_error 日志。");
            return 0;
        }
        catch (Exception error)
        {
            MessageBox.Show(error.Message, "UAV Motion Sim", MessageBoxButtons.OK, MessageBoxIcon.Error);
            return 1;
        }
    }
}
