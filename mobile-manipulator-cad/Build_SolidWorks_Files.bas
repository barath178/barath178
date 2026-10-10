' Build_SolidWorks_Files.bas
' Builds native SOLIDWORKS files (.SLDPRT parts + .SLDASM assemblies) from the
' STEP parts in this "solidworks" folder, without the STEP-assembly importer.
'
' How to run (SOLIDWORKS 2025):
'   1. Tools > Macro > New...  Save it as Build_SolidWorks_Files.swp inside this
'      "solidworks" folder. The VBA editor opens.
'   2. Delete everything in the editor, paste ALL of this file, press F5.
'   3. Check the folder shown in the box, click OK and wait (1-3 minutes).
'      If SOLIDWORKS asks "Run Import Diagnostics?", click No.
'
' Output:
'   UR5_Fixed_Manipulator\native\UR5_Fixed_Manipulator.SLDASM
'   TB3_OpenManipulatorX\native\TB3_OpenManipulatorX.SLDASM
'   Material_Handling_Cell\native\Material_Handling_Cell.SLDASM
'   plus every part as .SLDPRT and one sub-assembly per URDF link.
'
' Needs the default references of a new macro: SOLIDWORKS 2025 Type Library and
' SOLIDWORKS 2025 Constant type library (Tools > References in the VBA editor).

Option Explicit

Dim swApp As SldWorks.SldWorks
Dim gRoot As String
Dim gLog As String

Sub main()
    Set swApp = Application.SldWorks

    Dim macroDir As String
    macroDir = swApp.GetCurrentMacroPathName
    macroDir = Left(macroDir, InStrRev(macroDir, "\"))
    gRoot = InputBox("Folder that contains UR5_Fixed_Manipulator, TB3_OpenManipulatorX and Material_Handling_Cell:", _
                     "Build SOLIDWORKS files", macroDir)
    If gRoot = "" Then Exit Sub
    If Right(gRoot, 1) <> "\" Then gRoot = gRoot & "\"
    If Dir(gRoot & "UR5_Fixed_Manipulator\assembly_manifest.txt") = "" Then
        MsgBox "assembly_manifest.txt not found under" & vbCrLf & gRoot, vbExclamation
        Exit Sub
    End If
    If MsgBox("All open SOLIDWORKS documents will be closed. Save your work first." & vbCrLf & "Continue?", _
              vbOKCancel + vbQuestion) <> vbOK Then Exit Sub
    swApp.CloseAllDocuments True

    On Error GoTo Failed
    BuildRobot "UR5_Fixed_Manipulator"
    BuildRobot "TB3_OpenManipulatorX"
    BuildCell

    Dim e As Long, w As Long
    swApp.OpenDoc6 gRoot & "Material_Handling_Cell\native\Material_Handling_Cell.SLDASM", _
                   swDocASSEMBLY, swOpenDocOptions_Silent, "", e, w
    MsgBox "Done. Native SOLIDWORKS files are in each 'native' folder." & gLog, vbInformation
    Exit Sub
Failed:
    swApp.DocumentVisible True, swDocPART
    swApp.DocumentVisible True, swDocASSEMBLY
    MsgBox "Stopped: " & Err.Description & gLog, vbCritical
End Sub

Sub BuildRobot(robot As String)
    Dim base As String, outDir As String
    base = gRoot & robot & "\"
    outDir = base & "native\"
    If Dir(outDir, vbDirectory) = "" Then MkDir outDir

    Dim lines As Collection, linkAsms As New Collection, linkParts As Collection
    Set lines = ReadManifest(base & "assembly_manifest.txt")
    Dim i As Long, f() As String, curLink As String, prt As String
    For i = 1 To lines.Count
        f = Split(lines(i), ";")
        prt = outDir & f(1) & ".SLDPRT"
        StepToPart base & "parts\" & f(1) & ".STEP", prt
        If f(0) <> curLink Then
            If curLink <> "" Then linkAsms.Add MakeAssembly(linkParts, outDir & curLink & ".SLDASM")
            curLink = f(0)
            Set linkParts = New Collection
        End If
        linkParts.Add prt
    Next i
    If curLink <> "" Then linkAsms.Add MakeAssembly(linkParts, outDir & curLink & ".SLDASM")
    MakeAssembly linkAsms, outDir & robot & ".SLDASM"
End Sub

Sub BuildCell()
    Dim base As String, outDir As String
    base = gRoot & "Material_Handling_Cell\"
    outDir = base & "native\"
    If Dir(outDir, vbDirectory) = "" Then MkDir outDir

    Dim lines As Collection, items As New Collection
    Set lines = ReadManifest(base & "assembly_manifest.txt")
    Dim i As Long, f() As String, path As String
    For i = 1 To lines.Count
        f = Split(lines(i), ";")
        If f(0) = "robot" Then
            path = gRoot & f(1) & "\native\" & f(1) & ".SLDASM"
        Else
            path = outDir & f(1) & ".SLDPRT"
            StepToPart base & "parts\" & f(1) & ".STEP", path
        End If
        items.Add path & "|" & f(2) & "|" & f(3) & "|" & f(4)
    Next i
    MakeAssembly items, outDir & "Material_Handling_Cell.SLDASM"
End Sub

Function ReadManifest(path As String) As Collection
    Dim c As New Collection, fn As Integer, s As String
    If Dir(path) = "" Then Err.Raise vbObjectError + 1, , "Missing " & path
    fn = FreeFile
    Open path For Input As #fn
    Do While Not EOF(fn)
        Line Input #fn, s
        s = Trim(s)
        If Len(s) > 0 And Left(s, 1) <> "#" Then c.Add s
    Loop
    Close #fn
    Set ReadManifest = c
End Function

' Import one STEP part and save it as .SLDPRT
Sub StepToPart(stepPath As String, partPath As String)
    Dim errs As Long, warns As Long
    Dim imp As Object, doc As SldWorks.ModelDoc2
    If Dir(stepPath) = "" Then Err.Raise vbObjectError + 2, , "Missing " & stepPath
    Set imp = swApp.GetImportFileData(stepPath)
    Set doc = swApp.LoadFile4(stepPath, "r", imp, errs)
    If doc Is Nothing Then Err.Raise vbObjectError + 3, , "Could not import " & stepPath & " (error " & errs & ")"
    If doc.GetType <> swDocPART Then gLog = gLog & vbCrLf & "Not imported as a part: " & stepPath
    If Not doc.Extension.SaveAs(partPath, swSaveAsCurrentVersion, swSaveAsOptions_Silent, Nothing, errs, warns) Then
        Err.Raise vbObjectError + 4, , "Could not save " & partPath & " (error " & errs & ")"
    End If
    swApp.CloseDoc doc.GetTitle
End Sub

' New assembly with every item fixed in place. item = "path" or "path|x|y|z" (metres)
Function MakeAssembly(items As Collection, outPath As String) As String
    Dim tmpl As String, errs As Long, warns As Long
    Dim swModel As SldWorks.ModelDoc2, swAssy As SldWorks.AssemblyDoc, comp As SldWorks.Component2
    tmpl = swApp.GetUserPreferenceStringValue(swDefaultTemplateAssembly)
    Set swModel = swApp.NewDocument(tmpl, 0, 0, 0)
    If swModel Is Nothing Then Err.Raise vbObjectError + 5, , _
        "Could not create an assembly from template '" & tmpl & "'. Set it in Tools > Options > Default Templates."
    Set swAssy = swModel

    Dim it As Variant, p() As String, docType As Long, x As Double, y As Double, z As Double
    For Each it In items
        p = Split(CStr(it), "|")
        x = 0: y = 0: z = 0
        If UBound(p) = 3 Then x = Val(p(1)): y = Val(p(2)): z = Val(p(3))
        If UCase(Right(p(0), 7)) = ".SLDASM" Then docType = swDocASSEMBLY Else docType = swDocPART
        swApp.DocumentVisible False, docType
        swApp.OpenDoc6 p(0), docType, swOpenDocOptions_Silent, "", errs, warns
        swApp.DocumentVisible True, docType
        Set comp = swAssy.AddComponent5(p(0), swAddComponentConfigOptions_CurrentSelectedConfig, "", False, "", x, y, z)
        If comp Is Nothing Then
            gLog = gLog & vbCrLf & "Could not insert " & p(0)
        Else
            swModel.ClearSelection2 True
            comp.Select4 False, Nothing, False
            swAssy.FixComponent
        End If
    Next it
    swModel.ClearSelection2 True
    swModel.ForceRebuild3 False
    swModel.ShowNamedView2 "*Isometric", swIsometricView
    swModel.ViewZoomtofit2
    If Not swModel.Extension.SaveAs(outPath, swSaveAsCurrentVersion, swSaveAsOptions_Silent, Nothing, errs, warns) Then
        Err.Raise vbObjectError + 6, , "Could not save " & outPath & " (error " & errs & ")"
    End If
    swApp.CloseDoc swModel.GetTitle
    MakeAssembly = outPath
End Function
