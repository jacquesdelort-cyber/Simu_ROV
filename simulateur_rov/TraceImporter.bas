Option Explicit

Private Function SanitizeSheetName(ByVal name As String) As String
    Dim invalidChars As Variant
    Dim i As Integer
    Dim cleaned As String
    invalidChars = Array("\\", "/", "?", "*", "[", "]", ":")
    cleaned = name
    For i = LBound(invalidChars) To UBound(invalidChars)
        cleaned = Replace(cleaned, invalidChars(i), "_")
    Next i
    If Len(cleaned) = 0 Then
        cleaned = "Trace"
    End If
    If Len(cleaned) > 31 Then
        cleaned = Left(cleaned, 31)
    End If
    SanitizeSheetName = cleaned
End Function

Private Function NormalizeFilePath(ByVal pathIn As String) As String
    Dim prefix As String
    Dim root As String
    Dim parts As Variant
    Dim i As Long
    Dim rel As String
    
    prefix = "https://d.docs.live.net/"
    If LCase$(Left$(pathIn, Len(prefix))) = prefix Then
        root = Environ$("OneDrive")
        If Len(root) = 0 Then
            root = Environ$("OneDriveConsumer")
        End If
        If Len(root) = 0 Then
            NormalizeFilePath = pathIn
            Exit Function
        End If
        parts = Split(pathIn, "/")
        If UBound(parts) >= 4 Then
            rel = ""
            For i = 4 To UBound(parts)
                If Len(rel) = 0 Then
                    rel = parts(i)
                Else
                    rel = rel & "\" & parts(i)
                End If
            Next i
            rel = Replace(rel, "%20", " ")
            NormalizeFilePath = root & "\" & rel
            Exit Function
        End If
    End If
    NormalizeFilePath = pathIn
End Function

Public Sub ChargerTrace()
    Dim fd As FileDialog
    Dim filePath As String
    Dim sheetName As String
    Dim ws As Worksheet
    Dim qt As QueryTable

    Set fd = Application.FileDialog(msoFileDialogFilePicker)
    With fd
        .Title = "Charger trace"
        .Filters.Clear
        .Filters.Add "Fichiers CSV", "*.csv"
        .AllowMultiSelect = False
        .InitialFileName = ThisWorkbook.Path & "\\"
        If .Show <> -1 Then Exit Sub
        filePath = NormalizeFilePath(.SelectedItems(1))
    End With

    Dim fso As Object
    Dim fileName As String
    Set fso = CreateObject("Scripting.FileSystemObject")
    fileName = fso.GetBaseName(filePath)
    sheetName = SanitizeSheetName(fileName)

    Application.DisplayAlerts = False
    On Error Resume Next
    ThisWorkbook.Worksheets(sheetName).Delete
    On Error GoTo 0
    Application.DisplayAlerts = True

    Set ws = ThisWorkbook.Worksheets.Add(After:=ThisWorkbook.Worksheets(ThisWorkbook.Worksheets.Count))
    ws.Name = sheetName

    Dim f As Integer
    Dim firstLine As String
    Dim colCount As Long
    Dim fieldInfo() As Variant
    Dim i As Long

    f = FreeFile
    Open filePath For Input As #f
    Line Input #f, firstLine
    Close #f

    colCount = UBound(Split(firstLine, ",")) + 1
    If colCount < 1 Then Exit Sub
    ReDim fieldInfo(1 To colCount)
    For i = 1 To colCount
        fieldInfo(i) = 2 ' xlTextFormat
    Next i

    Set qt = ws.QueryTables.Add(Connection:="TEXT;" & filePath, Destination:=ws.Range("A1"))
    With qt
        .TextFileParseType = xlDelimited
        .TextFileCommaDelimiter = True
        .TextFileSemicolonDelimiter = False
        .TextFileTabDelimiter = False
        .TextFileOtherDelimiter = False
        .TextFileConsecutiveDelimiter = False
        .TextFileColumnDataTypes = fieldInfo
        .TextFilePlatform = 65001
        .Refresh BackgroundQuery:=False
    End With

    Dim used As Range
    Dim arr As Variant
    Dim r As Long
    Dim c As Long
    Dim s As String
    Dim norm As String

    Set used = ws.UsedRange
    If Not used Is Nothing Then
        arr = used.Value
        For r = 2 To UBound(arr, 1)
            For c = 1 To UBound(arr, 2)
                If VarType(arr(r, c)) = vbString Then
                    s = Trim$(arr(r, c))
                    If Len(s) > 0 Then
                        norm = Replace(s, ",", ".")
                        If IsNumeric(norm) Then
                            arr(r, c) = Val(norm)
                        Else
                            arr(r, c) = s
                        End If
                    End If
                End If
            Next c
        Next r
        used.Value = arr
    End If
End Sub
