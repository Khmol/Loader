from cx_Freeze import setup, Executable

setup(
    name = "Loader",
    version = "1.1",
    description = "Loader",
    executables = [Executable("c:\python34\Scripts\Loader\Loader.py")]
)

#C:\Python34\python.exe setup.py build
#C:\Python34\python.exe setup.py bdist_msi
