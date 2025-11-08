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

    _view.autoResizeDrawable = YES;
    NSScreen *scr = self.view.window.screen ?: NSScreen.mainScreen;

    _renderer = [[Renderer alloc] initWithMetalKitView:_view];
    NSAssert(_renderer, @"Renderer failed initialization");

    [_renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];
    _view.delegate = _renderer;
}

@end
