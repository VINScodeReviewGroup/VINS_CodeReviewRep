//  Thumbnail.m
//  VINS_ios
//
//  Created by 李健宽 on 2017/7/13.
//  Copyright © 2017年 李健宽. All rights reserved.
//

#import "Thumbnail.h"

@interface ThumbnailView()



@end

@implementation ThumbnailView

//TODO:画面宽高
const int width = 1280;
const int height = 1081;

-(void) drawRect:(CGRect)rect{
	
	CGContextRef context = UIGraphicsGetCurrentContext();
	
	//绘制圆形
	CGRect frame = CGRectMake(pos_x, pos_y, 10, 10);
	[[UIColor whiteColor] set];
	CGContextFillRect(context, rect);
	
	CGContextAddEllipseInRect(context, frame);
	[[UIColor orangeColor] set];
	CGContextFillPath(context);
	
}

- (void) setPixelX:(int)pixel_x{
	pos_x = pixel_x / 10.0;
	
}

- (void) setPixelY:(int)pixel_y{
	pos_y = pixel_y / 10.0;
}


@end
