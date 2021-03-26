/*
 * Rotation and Translation
 * Putting it all together
 */
import MetalKit
import PlaygroundSupport

let frameRect = CGRect(x: 0, y: 0,
                       width: 320, height: 320)
let device = MTLCreateSystemDefaultDevice()!
let view = MetalView(frame: frameRect,
                     device: device)
PlaygroundPage.current.liveView = view
