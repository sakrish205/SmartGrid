Set fso = CreateObject("Scripting.FileSystemObject")
Set sh  = CreateObject("WScript.Shell")
dir = fso.GetParentFolderName(WScript.ScriptFullName)
sh.Run "cmd /c cd /d """ & dir & """ && pythonw SmartGrid.pyw", 0
