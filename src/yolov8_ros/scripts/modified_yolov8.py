#!/usr/bin/env python3

from ultralytics import YOLO
import torch
import torch.nn as nn
from gsconv import GSConv, VoVGSCSP
from ultralytics.nn.modules import C2f
from ultralytics.models.yolo.model import Model as YOLOModel

class LightweightYOLOv8:
    """
    Implementation of the improved YOLOv8 with GSConv and VoVGSCSP modules
    as described in the paper.
    """
    def __init__(self, model_path='yolov8n-seg.pt'):
        """
        Initialize the modified YOLOv8 model
        
        Args:
            model_path: Path to the YOLOv8 weights
        """
        # Load the base model
        self.model = YOLO(model_path)
        
        # Modify the model architecture
        self._modify_model_architecture()
        
    def _modify_model_architecture(self):
        """
        Modify the YOLOv8 model architecture by:
        1. Replacing some Conv layers with GSConv
        2. Replacing some C2f modules with VoVGSCSP
        """
        # Access the internal PyTorch model
        model = self.model.model
        
        # Get the model structure
        backbone = model.model[0]  # Backbone
        neck = model.model[1]      # Neck
        
        # Replace Conv layers with GSConv in selected parts of the backbone
        # This is an example - you may need to adapt this based on the specific YOLOv8 model
        for i, layer in enumerate(backbone):
            if isinstance(layer, nn.Conv2d) and i > 4:  # Skip early layers
                c1, c2 = layer.in_channels, layer.out_channels
                k, s = layer.kernel_size[0], layer.stride[0]
                p = layer.padding[0]
                g = layer.groups
                
                # Replace with GSConv
                backbone[i] = GSConv(c1, c2, k, s, p, g)
        
        # Replace selected C2f modules with VoVGSCSP in the neck
        # Note: Based on the paper, we replace 3 C2f modules
        replacement_count = 0
        for i, layer in enumerate(neck):
            if isinstance(layer, C2f) and replacement_count < 3:
                c1, c2 = layer.cv1.in_channels, layer.cv2.out_channels
                e = 0.5  # Default expansion factor
                
                # Replace with VoVGSCSP
                neck[i] = VoVGSCSP(c1, c2, e=e)
                replacement_count += 1
        
        return model
        
    def predict(self, *args, **kwargs):
        """
        Run prediction with the modified model
        
        Args:
            *args, **kwargs: Arguments to pass to the model's predict method
        
        Returns:
            Prediction results
        """
        return self.model.predict(*args, **kwargs)
    
    def train(self, *args, **kwargs):
        """
        Train the modified model
        
        Args:
            *args, **kwargs: Arguments to pass to the model's train method
        
        Returns:
            Training results
        """
        return self.model.train(*args, **kwargs)
    
    def export(self, format='onnx'):
        """
        Export the modified model to the specified format
        
        Args:
            format: Export format (default: 'onnx')
            
        Returns:
            Path to the exported model
        """
        return self.model.export(format=format)