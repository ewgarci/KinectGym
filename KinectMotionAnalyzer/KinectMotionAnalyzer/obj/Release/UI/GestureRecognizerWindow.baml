   M S B A M L   `   `   `  ����   KinectMotionAnalyzer5    /KinectMotionAnalyzer.UI.GestureRecognizerWindow   W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     -     � Title$  GestureRecognizerWindow�� � SizeToContent$ Height=� � 
ResizeMode$ NoResize=�$
��1269��.��� .+��� -   $��	groupBox3��$��Display��$	��461��$
��Left=�$��	31,12,0,0q�$	��Top=�$	��452��$��16(�.��� .+��� -   $��color_disp_img��$��Uniform=�$	��413��$	��414��$��
11,6,16,13q��� -   $��ske_disp_img��$	��414��$��
11,6,16,13q�$��Uniform=�$	��413���� -   $��skeletonVideoSlider��$
��Left=� �� Interval$ 1��{�.   �� IsSnapToTickEnabled . $��75,479,0,159q�$	��345���� -   $��skeletonSliderLabel��$��1q�$��0��$��Normal&�$��32��$
��Left=�$��194,501,0,0q�$	��Top=�$	��126��$��Center=�$��13(�$��Center=��� -   $��kinectStatusBar�����	#FFB49788$��18(�$��43��$
��Left=�$��
12,606,0,0q�$	��Top=�$	��900��.@��� -   $��statusbarLabel��$��Welcome��$��Medium&�$��14(��� -	   $��replay_setStartBtn��$��	Set Start��$��34��$
��Left=�$��
81,547,0,0q�$	��Top=�$��92��$��14(��� -
   $��replay_setEndBtn��$��Set End��$��34��$
��Left=�$��261,547,0,0q�$	��Top=�$��92��$��14(��� -   $��replay_endLabel��$��1q�$��0��$��13(�$��Normal&�$��32��$
��Left=�$��Center=�$��362,549,0,0q�$	��Top=�$��Center=�$��68���� -   $��replay_startLabel��$��1q�$��0��$��13(�$��Normal&�$��32��$
��Left=�$��Center=�$��179,549,0,0q�$	��Top=�$��Center=�$��65���� -   $��
previewBtn��$��Preview Stream��$��15(�$��40��$
��Left=�$��
525,62,0,0q�$	��Top=�$	��182���� -   $��	groupBox1��$��Gesture Management��$	��316��$
��Left=�$��513,232,0,0q�$	��Top=�$	��342��$��16(�.��� .+��� -   $��gestureComboBox��$��34��$
��Left=�$��	41,22,0,0q�$	��Top=�$	��222�� �� Text$  �� J� SnapsToDevicePixels . $��Center=�$��Center=�$��15(�.@��� $��Choose Gesture��{�.   �� 
IsSelected . �� -   $��add_gesture_btn��$��Add Gesture��$��15(�$��40��$
��Left=�$��	21,77,0,0q�$	��Top=�$	��118���� -   $��remove_gesture_btn��$��Remove Gesture��$��15(�$��40��$
��Left=�$��
174,77,0,0q�$	��Top=�$	��118���� -   $��gestureCaptureBtn��$��Capture��$��44��$
��Left=�$��
21,0,0,104q�$��Bottom=�$	��118��{�. $��15(��� -   $��saveGestureBtn��$��Save Gesture��$��15(�$��44��$
��Left=�{�.  $��174,0,0,104q�$��Bottom=�$	��118���� -   $��gestureReplayBtn��$��Load Gesture��$��46��$
��Left=�$��105,213,0,0q�$	��Top=�$	��118��$��15(��� -   $��	groupBox2��$��Gesture Recognition��$	��354��$
��Left=�$��
881,40,0,0q�$	��Top=�$	��336��$��16(�.���  � ColumnDefinitions �� $	��51* ��� $
��233* �.+��� -   $��last_detection_label��$��Last Detection��$��16(�$��46��$
��Left=�$��Center=�$��	28,19,0,0q�$	��Top=�$��Center=�$	��271��$��2q�$��2���� -   $��rec_res_label��$��Current Detection��$��16(�$��46��$
��Left=�$��Center=�$��	28,71,0,0q�$	��Top=�$��Center=�$	��271��$��2���� -   $��gestureRecognitionBtn��$��Start Recognition��$��15(�$��43��$
��Left=�$��
23,129,0,0q�$	��Top=�$	��155��{�. $��1��� -   $��gesture_match_scorebar��$��29��$
��Left=�$��
28,214,0,0q�$	��Top=�$	��271��	 � Maximum$		 150Q�$��2���� -   $��recDistLabel��$��Matching Distance��$��46��$
��Left=�$��
23,263,0,0q�$	��Top=�$	��158��$��Center=�$��Center=�$��16(�$��1���� -   $��feedback_textblock��$��60��$
��Left=�$��525,146,0,0q�$��User Feedback��$	��Top=�$	��330��$
��Wrap=�
 �� TextAlignment$
 Center=�$��16(��� -   $��saveVideoCheckBox��$��
Save video��$��27��$
��Left=�$��
737,68,0,0q�$	��Top=�$	��112��$��16(�