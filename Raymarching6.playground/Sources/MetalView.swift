
import MetalKit

public class MetalView: MTKView, NSWindowDelegate {
    
    var queue: MTLCommandQueue! = nil
    var cps: MTLComputePipelineState! = nil
    var u_time: Float = 0
    var u_timeBuffer: MTLBuffer!
    var u_mouseBuffer: MTLBuffer!

    var inputTexture: MTLTexture!

    override public init(frame frameRect: CGRect,
                         device: MTLDevice?) {
        super.init(frame: frameRect,
                   device: device)
        queue = device!.makeCommandQueue()
        registerShaders()
    }

    required public init(coder: NSCoder) {
        super.init(coder: coder)
    }

    func registerShaders() {
        let path = Bundle.main.path(forResource: "Shaders",
									ofType: "metal")
        do {
            let input = try String(contentsOfFile: path!,
								   encoding: String.Encoding.utf8)
            let library = try device!.makeLibrary(source: input,
												  options: nil)
            let kernel = library.makeFunction(name: "compute")!
            cps = try device!.makeComputePipelineState(function: kernel)
        }
		catch let e {
            Swift.print("\(e)")
        }
        u_timeBuffer = device!.makeBuffer(length: MemoryLayout<Float>.stride,
                                          options: [])
        u_mouseBuffer = device!.makeBuffer(length: MemoryLayout<float2>.stride,
                                           options: [])
    }

    func createTexture(from fileName: String,
                       flip: Bool) {
        let comps = fileName.components(separatedBy:".")
        let path = Bundle.main.path(forResource: comps[0],
                                    ofType: comps[1])
        let nsImage = NSImage(contentsOfFile: path!)!
        let cgImage = nsImage.cgImage(forProposedRect: nil, context: nil, hints: nil)

        let colorSpace = CGColorSpaceCreateDeviceRGB()

        let bitsPerComponent = cgImage!.bitsPerComponent
        let bytesPerRow = cgImage!.bytesPerRow

        // The function will allocate memory
        let context = CGContext(data: nil,
                                width: cgImage!.width, height: cgImage!.height,
                                bitsPerComponent: bitsPerComponent,
                                bytesPerRow: bytesPerRow,
                                space: colorSpace,
                                bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
        let bounds = CGRect(x: 0, y: 0,
                            width: Int(cgImage!.width), height: Int(cgImage!.height))
        context.clear(bounds)       // paints a transparent rect.
        
        if flip == true {
            context.translateBy(x: 0, y: CGFloat(cgImage!.height))
            context.scaleBy(x: 1.0, y: -1.0)
        }
        context.draw(cgImage!, in: bounds)
        
        let texDescriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: MTLPixelFormat.rgba8Unorm,
                                                                     width: Int(cgImage!.width),
                                                                     height: Int(cgImage!.height),
                                                                     mipmapped: false)
        inputTexture = device!.makeTexture(descriptor: texDescriptor)!
        
        let pixelsData = context.data   // pointer to the image data
        let region = MTLRegionMake2D(0, 0, Int(cgImage!.width), Int(cgImage!.height))
        inputTexture.replace(region: region,
                             mipmapLevel: 0,
                             withBytes: pixelsData!,
                             bytesPerRow: Int(bytesPerRow))
    }

    override public func mouseDown(with event: NSEvent) {
        var mouseLocation = self.convert(event.locationInWindow, from: nil)
        let bufferPointer = u_mouseBuffer.contents()
        memcpy(bufferPointer, &mouseLocation, MemoryLayout<float2>.stride)
    }

    func updateTime() {
        u_time += 0.01
        let bufferPointer = u_timeBuffer.contents()
        memcpy(bufferPointer, &u_time, MemoryLayout<Float>.stride)
    }

    override public func draw(_ dirtyRect: NSRect) {
        updateTime()
        if let drawable = currentDrawable,
           let commandBuffer = queue.makeCommandBuffer(),
           let commandEncoder = commandBuffer.makeComputeCommandEncoder() {
            commandEncoder.setComputePipelineState(cps)
            commandEncoder.setBuffer(u_timeBuffer,
                                     offset: 0,
                                     index: 0)
            commandEncoder.setBuffer(u_mouseBuffer,
                                     offset: 0,
                                     index: 1)

            commandEncoder.setTexture(drawable.texture,
                                      index: 0)
            commandEncoder.setTexture(inputTexture,
                                      index: 1)

            let width = cps.threadExecutionWidth
            let height = cps.maxTotalThreadsPerThreadgroup / width
            let threadsPerGroup = MTLSizeMake(width, height, 1)
            let threadsPerGrid = MTLSizeMake(drawable.texture.width,
                                             drawable.texture.height,
                                             1)
            commandEncoder.dispatchThreads(threadsPerGrid,
                                           threadsPerThreadgroup: threadsPerGroup)
            commandEncoder.endEncoding()
            commandBuffer.present(drawable)
            commandBuffer.commit()
        }
    }
}
