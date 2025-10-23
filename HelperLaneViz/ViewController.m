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
}

- (void)viewDidLoad {
    [super viewDidLoad];

    _view = (MTKView *)self.view;
    _view.device = MTLCreateSystemDefaultDevice();

    _renderer = [[Renderer alloc] initWithMetalKitView:_view];
    NSAssert(_renderer, @"Renderer failed initialization");

    [_renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];
    _view.delegate = _renderer;

    _tilesButton = [NSButton buttonWithTitle:@"Tiles"
                                         target:self
                                      action:@selector(runTileProbe:)];
       _tilesButton.bezelStyle = NSBezelStyleRounded;
       _tilesButton.translatesAutoresizingMaskIntoConstraints = NO;
       [self.view addSubview:_tilesButton];

       [NSLayoutConstraint activateConstraints:@[
           [_tilesButton.topAnchor constraintEqualToAnchor:self.view.topAnchor constant:8],
           [_tilesButton.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:8]
       ]];
}

- (IBAction)runTileProbe:(id)sender {
    _tilesButton.enabled = NO;
    [_renderer measureTilesNow];
    _tilesButton.enabled = YES;
}

@end
