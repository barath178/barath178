Attribute VB_Name = "BuildSolidWorksAssembly"
' ============================================================================
'  Myoelectric hand -> native SolidWorks files (written for SolidWorks 2025)
'
'  1. Converts every  parts\*.step  into  native_sldworks\<name>.SLDPRT
'  2. Builds          native_sldworks\Myoelectric Hand Assembly.SLDASM
'     from instances.csv (60 components, each placed with its exact
'     rotation + position and then Fixed).
'
'  HOW TO RUN
'   Tools > Macro > New...  -> save e.g. hand.swp anywhere
'   In the VBA editor: File > Import File... -> pick this .bas file
'   Delete the empty default module, then run  Main  (F5).
'   When asked, select the "solidworks" folder that contains instances.csv.
'   If SolidWorks offers "Import Diagnostics" for a STEP file, answer No.
' ============================================================================
Option Explicit

Const swDocPART As Long = 1
Const swDocASSEMBLY As Long = 2
Const swOpenDocOptions_Silent As Long = 1
Const swSaveAsCurrentVersion As Long = 0
Const swSaveAsOptions_Silent As Long = 1
Const swDefaultTemplateAssembly As Long = 9

Dim swApp As Object

Sub Main()
    Set swApp = Application.SldWorks

    Dim baseDir As String
    baseDir = PickFolder()
    If baseDir = "" Then Exit Sub
    If Right(baseDir, 1) <> "\" Then baseDir = baseDir & "\"
    If Dir(baseDir & "instances.csv") = "" Then
        MsgBox "instances.csv not found in " & baseDir & vbCrLf & "Select the 'solidworks' folder.", vbExclamation
        Exit Sub
    End If

    Dim outDir As String
    outDir = baseDir & "native_sldworks\"
    If Dir(outDir, vbDirectory) = "" Then MkDir outDir

    ' ---------- 1. STEP -> SLDPRT
    Dim f As String, nOk As Long, nFail As Long, failed As String
    f = Dir(baseDir & "parts\*.step")
    Do While f <> ""
        If ConvertPart(baseDir & "parts\" & f, outDir & Left(f, Len(f) - 5) & ".SLDPRT") Then
            nOk = nOk + 1
        Else
            nFail = nFail + 1: failed = failed & vbCrLf & f
        End If
        f = Dir()
    Loop
    If nFail > 0 Then
        MsgBox nFail & " part(s) failed to convert:" & failed, vbExclamation
        Exit Sub
    End If

    ' ---------- 2. Assembly
    Dim tmpl As String
    tmpl = swApp.GetUserPreferenceStringValue(swDefaultTemplateAssembly)
    Dim asmDoc As Object
    Set asmDoc = swApp.NewDocument(tmpl, 0, 0, 0)
    If asmDoc Is Nothing Then
        MsgBox "Could not create an assembly from the default template:" & vbCrLf & tmpl, vbCritical
        Exit Sub
    End If

    Dim mu As Object
    Set mu = swApp.GetMathUtility

    Dim fh As Integer, ln As String, c() As String, nComp As Long, errs As Long, warns As Long
    fh = FreeFile
    Open baseDir & "instances.csv" For Input As #fh
    Line Input #fh, ln                                   ' header
    Do While Not EOF(fh)
        Line Input #fh, ln
        If Len(Trim(ln)) > 0 Then
            c = Split(ln, ",")
            ' part_file,instance,r11..r33,x_mm,y_mm,z_mm,type
            Dim partPath As String
            partPath = outDir & c(0) & ".SLDPRT"
            swApp.OpenDoc6 partPath, swDocPART, swOpenDocOptions_Silent, "", errs, warns
            swApp.ActivateDoc3 asmDoc.GetTitle, False, 0, errs

            Dim comp As Object
            Set comp = asmDoc.AddComponent5(partPath, 0, "", False, "", 0, 0, 0)
            If comp Is Nothing Then
                Close #fh
                MsgBox "Could not insert " & partPath, vbCritical
                Exit Sub
            End If

            ' SolidWorks transforms use row vectors (p' = p*A + t), so A = transpose of the CSV matrix.
            Dim t(15) As Double, i As Integer, j As Integer
            For i = 0 To 2
                For j = 0 To 2
                    t(i * 3 + j) = Val(c(2 + j * 3 + i))
                Next j
            Next i
            t(9) = Val(c(11)) / 1000#: t(10) = Val(c(12)) / 1000#: t(11) = Val(c(13)) / 1000#   ' mm -> m
            t(12) = 1#
            comp.Transform2 = mu.CreateTransform(t)

            asmDoc.ClearSelection2 True
            comp.Select4 False, Nothing, False
            asmDoc.FixComponent
            nComp = nComp + 1
        End If
    Loop
    Close #fh

    asmDoc.ClearSelection2 True
    asmDoc.ForceRebuild3 False
    asmDoc.ViewZoomtofit2
    asmDoc.Extension.SaveAs outDir & "Myoelectric Hand Assembly.SLDASM", swSaveAsCurrentVersion, swSaveAsOptions_Silent, Nothing, errs, warns

    ' close the part windows that were opened for insertion
    f = Dir(outDir & "*.SLDPRT")
    Do While f <> ""
        swApp.CloseDoc f
        f = Dir()
    Loop

    MsgBox nOk & " parts converted and " & nComp & " components placed." & vbCrLf & _
           "Saved to: " & outDir & "Myoelectric Hand Assembly.SLDASM", vbInformation
End Sub

Private Function ConvertPart(stepPath As String, sldprtPath As String) As Boolean
    Dim importData As Object, errs As Long, warns As Long, doc As Object
    Set importData = swApp.GetImportFileData(stepPath)
    Set doc = swApp.LoadFile4(stepPath, "r", importData, errs)
    If doc Is Nothing Then ConvertPart = False: Exit Function
    If doc.GetType <> swDocPART Then
        swApp.CloseDoc doc.GetTitle
        ConvertPart = False: Exit Function
    End If
    ConvertPart = doc.Extension.SaveAs(sldprtPath, swSaveAsCurrentVersion, swSaveAsOptions_Silent, Nothing, errs, warns)
    swApp.CloseDoc doc.GetTitle
End Function

Private Function PickFolder() As String
    Dim sh As Object, fo As Object
    Set sh = CreateObject("Shell.Application")
    Set fo = sh.BrowseForFolder(0, "Select the 'solidworks' folder (contains instances.csv and parts\)", 0)
    If fo Is Nothing Then PickFolder = "" Else PickFolder = fo.Self.Path
End Function
