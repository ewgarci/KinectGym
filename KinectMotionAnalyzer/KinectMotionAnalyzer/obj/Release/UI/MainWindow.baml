   M S B A M L   `   `   `  ����   KinectMotionAnalyzer(    "KinectMotionAnalyzer.UI.MainWindow   W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -     � Title$  KinectPreview��$	��674��$
��1224��.���  � ShowGridLines .  $	��598��$
��1185�� � ColumnDefinitions �� $
��585* ��� $
��600* � � RowDefinitions �� $��20 ��� $	��100 ��� $��20 ��� $	��395 ��� $��13 ��� $	��10* �.+��� -   $��border1��
�������$��1q�$��20��$
��Left=�$	��Top=�$
��1181�����	#FF9AE81E$��2���� -   $��	groupBox1��$��1��$��Streams��$��72��$
��Left=�$��	24,13,0,0q�$	��Top=�$	��296��.��� .+��� -   $��colorCheckBox��$��Color��$��16��$
��Left=�$��	24,18,0,0q�$	��Top=� d� 	IsChecked$
 TrueY��� -   $��depthCheckBox��$��Depth��$��16��$
��Left=�$��
110,18,0,0q�$	��Top=��� -   $��skeletonCheckBox��$��Skeleton��$��16��$
��Left=�$��
196,18,0,0q�$	��Top=��� -   $��runBtn��$��1��$��Start Kinect��$��35��$
��Left=�$��
352,38,0,0q�$	��Top=�$	��115���� -   $��saveBtn��$��1��$��Screen shot��$��35��$��Right=�$��
0,38,573,0q�$	��Top=�$	��115��$��2���� -	   $��button1��$	��New��$��1��$��1��$��35��$
��Left=�$��
364,38,0,0q�$	��Top=�$��80���� -
   $��loadSkeletonBtn��$
��Load��$��35��$
��Left=�$��
462,38,0,0q�$	��Top=�$��80��$��1��$��1���� -   $��border2��$��2�����	#FF9AE81E$��1q�$
��Left=�$
��1181��$��2���� -   $��color_disp_img��$��3��$	��338��$	��271��$
��Left=�$��
30,24,0,98q�$��Uniform=��� -   $��depth_disp_img��$��3��$	��271��$
��Left=�$��406,26,0,98q�$��Uniform=�$	��365��$��2���� -   $��skeleton_disp_img��$��3��$	��271��$
��Left=�$��211,25,0,98q�$��Uniform=�$	��365��$��1���� -   $��kinectStatusBar��$��4��$��39��$
��Left=�$��	12,13,0,0q�$	��Top=�$
��1157��$��18(�$��2��$��2�����	#FFB49788.@��� -   $��statusbarLabel��.��	Welcome�� -   $��skeletonVideoSlider��$��1��$��3��$��23��$
��Left=�$��211,319,0,0q�$	��Top=�$	��365�� �� Interval$ 1�� �� IsSnapToTickEnabled . {�.  �� -   $��skeletonSliderLabel��$��0��$��1��$��3��$��31��$
��Left=�$��377,348,0,0q�$	��Top=�$��Normal&�$��0q��� -   $��startSkeletonRecordBtn��$��Start Record��$��35��$
��Left=�$��	81,38,0,0q�$	��Top=�$��80��$��1��$��1��{�.  �� -   $��stopSkeletonRecordBtn��$��Stop Record��$��35��$
��Left=�$��
179,38,0,0q�$	��Top=�$��80��$��1��$��1��{�.  