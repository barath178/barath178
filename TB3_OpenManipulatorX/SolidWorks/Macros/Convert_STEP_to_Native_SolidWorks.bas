Attribute VB_Name = "Convert_STEP_to_Native"
' ============================================================================
'  TB3 + OpenMANIPULATOR-X  :  STEP  ->  native SolidWorks 2025 (.SLDPRT / .SLDASM)
'
'  HOW TO RUN (SolidWorks 2025):
'    1. Tools > Macro > New...  save as  SolidWorks\Macros\Convert.swp  (this folder)
'    2. In the VBA editor: File > Import File...  pick this .bas file
'    3. Delete the empty "main" module SolidWorks created, then Run (F5) -> main
'  Result: SolidWorks\Native_SolidWorks_Files\Parts\*.SLDPRT
'          SolidWorks\Native_SolidWorks_Files\Assembly\*.SLDASM (+ their parts)
' ============================================================================
Option Explicit

Const swDocPART As Long = 1
Const swDocASSEMBLY As Long = 2
Const swSaveAsCurrentVersion As Long = 0
Const swSaveAsOptions_Silent As Long = 1
Const swSaveAsOptions_SaveReferenced As Long = 4

Dim swApp As Object

Sub main()
    Set swApp = Application.SldWorks
    Dim macroDir As String, root As String, outDir As String
    macroDir = swApp.GetCurrentMacroPathFolder
    If Right(macroDir, 1) = "\" Then macroDir = Left(macroDir, Len(macroDir) - 1)
    root = Left(macroDir, InStrRev(macroDir, "\") - 1)   ' ...\SolidWorks
    outDir = root & "\Native_SolidWorks_Files"
    MakeDir outDir

    Dim nP As Long, nA As Long
    nP = ConvertFolder(root & "\Parts", outDir & "\Parts")
    nA = ConvertFolder(root & "\Assembly", outDir & "\Assembly")
    MsgBox "Converted " & nP & " parts and " & nA & " assemblies into:" & vbCrLf & outDir, vbInformation
End Sub

Private Sub MakeDir(p As String)
    If Dir(p, vbDirectory) = "" Then MkDir p
End Sub

Private Function ConvertFolder(src As String, dst As String) As Long
    MakeDir dst
    Dim files() As String, n As Long, f As String, i As Long
    n = 0
    f = Dir(src & "\*.STEP")
    Do While f <> ""
        ReDim Preserve files(n)
        files(n) = f
        n = n + 1
        f = Dir()
    Loop
    ConvertFolder = 0
    If n = 0 Then Exit Function

    For i = 0 To n - 1
        Dim srcPath As String, baseName As String, outPath As String
        Dim imp As Object, doc As Object
        Dim errs As Long, warns As Long, ok As Boolean
        srcPath = src & "\" & files(i)
        baseName = Left(files(i), InStrRev(files(i), ".") - 1)
        Set imp = swApp.GetImportFileData(srcPath)
        Set doc = swApp.LoadFile4(srcPath, "r", imp, errs)
        If Not doc Is Nothing Then
            If doc.GetType = swDocASSEMBLY Then
                outPath = dst & "\" & baseName & ".SLDASM"
            Else
                outPath = dst & "\" & baseName & ".SLDPRT"
            End If
            ok = doc.Extension.SaveAs3(outPath, swSaveAsCurrentVersion, _
                    swSaveAsOptions_Silent + swSaveAsOptions_SaveReferenced, Nothing, Nothing, errs, warns)
            If ok Then ConvertFolder = ConvertFolder + 1 Else Debug.Print "Save failed: " & outPath & " err=" & errs
            swApp.CloseAllDocuments True
        Else
            Debug.Print "Could not open: " & srcPath & " err=" & errs
        End If
    Next i
End Function
