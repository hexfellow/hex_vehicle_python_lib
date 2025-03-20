import os
from setuptools import setup, find_packages
from setuptools.command.build_py import build_py as _build_py
import subprocess

class build_py(_build_py):
    def run(self):
        # 获取项目根目录的绝对路径
        project_dir = os.path.abspath(os.path.dirname(__file__))
        proto_dir = os.path.join(project_dir, 'proto-public-api')
        out_dir = os.path.join(project_dir, 'my_project', 'generated')
        
        if not os.path.exists(proto_dir):
            raise FileNotFoundError(f"Proto目录不存在: {proto_dir}")
        
        # 确保输出目录存在
        os.makedirs(out_dir, exist_ok=True)

        # 编译所有.proto文件
        proto_files = [f for f in os.listdir(proto_dir) if f.endswith('.proto')]
        for proto_file in proto_files:
            proto_path = os.path.join(proto_dir, proto_file)
            subprocess.check_call([
                'protoc',
                '--python_out', out_dir,
                '--proto_path', proto_dir,
                proto_path
            ])
        
        # 必须调用父类方法以继续常规构建流程
        super().run()

setup(
    name="my_project",
    version="0.1",
    packages=find_packages(),
    cmdclass={"build_py": build_py},
    include_package_data=True,
    package_data={
        '': ['proto-public-api/*.proto'],  # 包含proto文件
    },
    # 其他参数...
)