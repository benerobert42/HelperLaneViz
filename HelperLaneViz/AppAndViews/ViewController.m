//
//  ViewController.m
//  HelperLaneViz
//
//  Created by Robert Bene on 2025. 08. 16..
//

#import "MetalKit/MetalKit.h"
#import "ViewController.h"
#import "Renderer.h"

@implementation ViewController
{
    MTKView *_view;
    Renderer *_renderer;
    NSButton *_tilesButton;
    NSButton *_measureButton;
}

- (void)viewDidLoad {
    [super viewDidLoad];

    _view = (MTKView *)self.view;
    _view.device = MTLCreateSystemDefaultDevice();

    _view.autoResizeDrawable = YES;

    _renderer = [[Renderer alloc] initWithMetalKitView:_view];
    NSAssert(_renderer, @"Renderer failed initialization");

    [_renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];
    _view.delegate = _renderer;
    
    [self setupMeasureButton];
}

- (void)setupMeasureButton {
    _measureButton = [NSButton buttonWithTitle:@"⏱ Measure 100 Frames"
                                        target:self
                                        action:@selector(measureButtonClicked:)];
    _measureButton.bezelStyle = NSBezelStyleRounded;
    _measureButton.translatesAutoresizingMaskIntoConstraints = NO;
    [self.view addSubview:_measureButton];
    
    [NSLayoutConstraint activateConstraints:@[
        [_measureButton.topAnchor constraintEqualToAnchor:self.view.topAnchor constant:10],
        [_measureButton.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:-10]
    ]];
}

- (void)measureButtonClicked:(id)sender {
    if ([_renderer isFrameTimeMeasurementActive]) {
        return; // Measurement already in progress
    }
    [_renderer startFrameTimeMeasurement:400];
}

@end
