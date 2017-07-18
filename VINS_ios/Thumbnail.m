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
const int width = 2960;
const int height = 2500;

-(void) drawRect:(CGRect)rect{
	
	CGContextRef context = UIGraphicsGetCurrentContext();
	
	//绘制圆形
	CGRect frame = CGRectMake(pos_x, pos_y, 10, 10);
	[[UIColor clearColor] set];
	CGContextFillRect(context, rect);
	
	CGContextAddEllipseInRect(context, frame);
	[[UIColor orangeColor] set];
	CGContextFillPath(context);
	NSLog(@"wrz40 drawing...");
}

- (void) setPixelX:(int)pixel_x{
	pos_x = 1.0 * pixel_x /width * 156;
	
}

- (void) setPixelY:(int)pixel_y{
	pos_y = 1.0 * pixel_y / height * 132;
}


@end
