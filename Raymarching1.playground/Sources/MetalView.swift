
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
